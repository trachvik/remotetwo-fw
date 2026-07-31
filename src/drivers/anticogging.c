/*
 * anticogging.c
 * Final Consistent Calibration (Lerp + Single-Pass Filter)
 */

#include "anticogging.h"
#include "drivers/bldc_motor.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <math.h>

extern float sensor_get_angle(void *sensor);
extern void sensor_update(void *sensor);

LOG_MODULE_REGISTER(anticogging, LOG_LEVEL_INF);

#define SAMPLE_COUNT 500
#define TWO_PI 6.28318530718f
#define CALIB_SPEED_RAD_S 3.14159f
#define CALIB_REVOLUTIONS 3
#define MAX_ITERATIONS 5
#define RELAXATION_GAIN 0.5f
#define ERROR_THRESHOLD 0.005f 

static float final_lut[SAMPLE_COUNT] = {0};

float anticogging_get_feedforward(float mechanical_angle) 
{
    if (isnan(mechanical_angle) || isinf(mechanical_angle)) return 0.0f;
    
    float norm_angle = fmodf(mechanical_angle, TWO_PI);
    if (norm_angle < 0.0f) norm_angle += TWO_PI;
    
    float float_index = (norm_angle / TWO_PI) * (float)SAMPLE_COUNT;
    int idx1 = (int)float_index;
    
    /* BEZPEČNOST: Ochrana proti přetečení indexu na hranici otáčky */
    if (idx1 >= SAMPLE_COUNT) idx1 = SAMPLE_COUNT - 1;
    if (idx1 < 0) idx1 = 0;
    
    int idx2 = (idx1 + 1) % SAMPLE_COUNT; 
    float fraction = float_index - (float)idx1;
    
    return final_lut[idx1] * (1.0f - fraction) + final_lut[idx2] * fraction;
}

/* Circular (wrap-around) moving-average smoothing of a LUT in place. `window`
 * should be odd. Used to reject random per-bin spikes so the feedforward - and
 * therefore the motor motion - stays smooth. */
static void smooth_lut_circular(float *arr, int window)
{
    static float tmp[SAMPLE_COUNT];
    int half = window / 2;
    int n = 2 * half + 1;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        float sum = 0.0f;
        for (int j = -half; j <= half; j++) {
            int idx = (i + j + SAMPLE_COUNT) % SAMPLE_COUNT;
            sum += arr[idx];
        }
        tmp[i] = sum / (float)n;
    }
    for (int i = 0; i < SAMPLE_COUNT; i++) arr[i] = tmp[i];
}

static void run_directional_calibration(bldc_motor_t *motor, struct k_sem *foc_sem, float target_speed, float *output_lut) 
{
    static float temp_sum[SAMPLE_COUNT];
    static uint16_t temp_count[SAMPLE_COUNT];

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        temp_sum[i] = 0.0f;
        temp_count[i] = 0;
    }

    uint32_t stabilize_ticks = (uint32_t)(2.0f * 5000.0f); 
    int move_divider = 0;
    
    for(uint32_t i = 0; i < stabilize_ticks; i++) {
        k_sem_take(foc_sem, K_FOREVER);
        sensor_update(motor->sensor);
        
        if (++move_divider >= 5) { // 1000 Hz update for motion control
            move_divider = 0;
            bldc_motor_move(motor, target_speed);
        }
        
        float ff = anticogging_get_feedforward(sensor_get_angle(motor->sensor));
        float original_sp = motor->current_sp;
        motor->current_sp += ff; 
        bldc_motor_loop_foc(motor);
        motor->current_sp = original_sp; 
    }

    float time_needed_s = (CALIB_REVOLUTIONS * TWO_PI) / fabs(target_speed);
    uint32_t calib_ticks = (uint32_t)(time_needed_s * 5000.0f);

    for(uint32_t i = 0; i < calib_ticks; i++) {
        k_sem_take(foc_sem, K_FOREVER);
        sensor_update(motor->sensor);
        
        if (++move_divider >= 5) { // 1000 Hz update for motion control
            move_divider = 0;
            bldc_motor_move(motor, target_speed);
        }

        float angle = sensor_get_angle(motor->sensor);
        float ff = anticogging_get_feedforward(angle);
        
        float original_sp = motor->current_sp;
        motor->current_sp += ff; 
        bldc_motor_loop_foc(motor);
        motor->current_sp = original_sp; 

        float norm_angle = fmodf(angle, TWO_PI);
        if (norm_angle < 0.0f) norm_angle += TWO_PI;
        int index = (int)((norm_angle / TWO_PI) * SAMPLE_COUNT);
        if (index < 0) index = 0;
        if (index >= SAMPLE_COUNT) index = SAMPLE_COUNT - 1;

        temp_sum[index] += original_sp;
        temp_count[index]++;
    }

    bldc_motor_move(motor, 0.0f);
    bldc_motor_loop_foc(motor);

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        if (temp_count[i] > 0) {
            output_lut[i] = temp_sum[i] / (float)temp_count[i];
        } else {
            output_lut[i] = (i > 0) ? output_lut[i - 1] : 0.0f;
        }
    }
}

void anticogging_calibrate(bldc_motor_t *motor, struct k_sem *foc_sem) 
{
    /* 1. BEZPEČNOSTNÍ POJISTKA: Kontrola FOC Inicializace */
    if (motor->motor_status != MOTOR_READY) {
        printk("===================================================\n");
        printk("!!! ANTICOGGING CALIBRATION BLOCKED !!!\n");
        printk("FOC initialization (PP check) failed at startup.\n");
        printk("Motor is likely physically blocked or disconnected.\n");
        printk("Please free the motor shaft and restart the board.\n");
        printk("===================================================\n");
        return; 
    }

    static float lut_cw[SAMPLE_COUNT];
    static float lut_ccw[SAMPLE_COUNT];
    static float residual[SAMPLE_COUNT];

    float saved_kp = motor->pid_velocity.p;
    float saved_ki = motor->pid_velocity.i;
    float saved_v_limit = motor->pid_velocity.limit;
    float saved_c_limit = motor->current_limit;
    float saved_vel_ts = motor->lpf_velocity.Ts;

    /* OPRAVA: Návrat k "měkké gumě" bez Integrator Windup efektu */
    motor->pid_velocity.p = 0.05f;       /* Hladké obkreslení zubů (žádné kmitání) */
    motor->pid_velocity.i = 0.02f;       /* Miniaturní I-složka jen proti zastavení na tření */
    motor->pid_velocity.limit = 1.0f;    
    motor->current_limit = 1.0f;     

    /* CRITICAL: move() below runs at 1000 Hz (every 5 FOC ticks), but the runtime
     * config hard-codes lpf_velocity.Ts = 0.01 (100 Hz). That 10x mismatch leaves
     * the velocity estimate under-filtered and noisy - a primary cause of the
     * random oscillations. Match Ts to the real update rate so the tf=0.015 s
     * smoothing behaves as intended. Restored after calibration. */
    motor->lpf_velocity.Ts = 0.001f;

    for (int i = 0; i < SAMPLE_COUNT; i++) final_lut[i] = 0.0f;
    
    printk("---> ANTICOGGING CALIBRATION START\n");

    for (int iter = 1; iter <= MAX_ITERATIONS; iter++) {
        printk("---> ITERATION %d / %d ---\n", iter, MAX_ITERATIONS);

        run_directional_calibration(motor, foc_sem, CALIB_SPEED_RAD_S, lut_cw);
        k_msleep(500); 
        run_directional_calibration(motor, foc_sem, -CALIB_SPEED_RAD_S, lut_ccw);

        float max_residual_error = 0.0f;
        float sum_residual_error = 0.0f;

        /* Combine both directions (cancels direction-dependent friction, keeps
         * position-dependent cogging), clamp obvious spikes, then spatially
         * smooth the residual BEFORE feeding it back. A noisy residual is what
         * makes the feedforward - and thus the motion - jerky and oscillate. */
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            float r = 0.5f * (lut_cw[i] + lut_ccw[i]);
            if (r >  0.5f) r =  0.5f;
            if (r < -0.5f) r = -0.5f;
            residual[i] = r;
        }
        smooth_lut_circular(residual, 5);

        for (int i = 0; i < SAMPLE_COUNT; i++) {
            final_lut[i] += residual[i] * RELAXATION_GAIN;

            float abs_err = fabs(residual[i]);
            sum_residual_error += abs_err;
            if (abs_err > max_residual_error) max_residual_error = abs_err;
        }

        /* Light smoothing of the accumulated map after EACH iteration keeps the
         * feedforward used by the NEXT iteration clean, so the motion gets
         * smoother as it converges instead of noisier. */
        smooth_lut_circular(final_lut, 3);

        float mean_error = sum_residual_error / (float)SAMPLE_COUNT;
        printk("     Mean residual error = %.4f A (Max spike = %.4f A)\n", (double)mean_error, (double)max_residual_error);

        if (max_residual_error < ERROR_THRESHOLD) {
            printk("---> Error is below threshold. Calibration finished early!\n");
            break; 
        }
        k_msleep(500); 
    }

    /* =========================================================================
     * 3. FILTRACE: závěrečné lehké vyhlazení plně sestavené finální mapy
     * ========================================================================= */
    smooth_lut_circular(final_lut, 3);

    /* Návrat uživatelských PID hodnot po úspěšné kalibraci */
    motor->pid_velocity.p = saved_kp;
    motor->pid_velocity.i = saved_ki;
    motor->pid_velocity.limit = saved_v_limit;
    motor->current_limit = saved_c_limit;
    motor->lpf_velocity.Ts = saved_vel_ts;

    printk("---> CALIBRATION COMPLETED ---\n");
    printk("=== START LUT DUMP ===\n");
    printk("Index,Current_A\n");
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        printk("%d,%f\n", i, (double)final_lut[i]);
    }
    printk("=== END LUT DUMP ===\n");
    k_msleep(1000);
}

/* pozn:
   motor vzdy pri "nabehu" ma takovy sekany pohyb, ale po par otackach se to zlepsi (zkontrolovat cim to je, mozna se ty prvni tri otacky na prazdno nekompenzuji?)
   na druhou stranu nahodne oscilac zmizely, jak je videt v logu, tak chyba se ale opet s kadou iteraci mirne zvetsuje, to e taky musi nejak vyresit
 */

 /* log:
    *** Booting nRF Connect SDK v3.2.2-74845e169be2 ***
*** Using Zephyr OS v4.2.99-fe4f0106803e ***
[00:00:00.644,836] <inf> foc_current_test: ===========================================
[00:00:00.644,866] <inf> foc_current_test:  STARTING STANDALONE FOC CURRENT TEST
[00:00:00.644,866] <inf> foc_current_test: ===========================================
[00:00:00.644,866] <inf> foc_current_test: 1. Initializing TMAG5170 encoder...
[00:00:00.644,866] <inf> tmag5170_sensor: TMAG5170 encoder initialized
[00:00:00.645,355] <inf> tmag5170_sensor: TMAG diag[0]: Bx=-64.001 By= 48.903 |Bxy|=80.546 mT  angle= 143.000 deg
[00:00:00.647,857] <inf> tmag5170_sensor: TMAG diag[1]: Bx=-64.006 By= 49.002 |Bxy|=80.610 mT  angle= 142.750 deg
[00:00:00.650,329] <inf> tmag5170_sensor: TMAG diag[2]: Bx=-64.101 By= 48.907 |Bxy|=80.628 mT  angle= 142.750 deg
[00:00:00.652,801] <inf> tmag5170_sensor: TMAG diag[3]: Bx=-64.206 By= 49.007 |Bxy|=80.772 mT  angle= [00:00:01.587,646] <inf> foc_current_test: 4. Running FOC calibration...
[00:00:02.503,295] <inf> bldc_motor: MOT: Align sensor.
[00:00:05.163,879] <inf> bldc_motor: MOT: sensor_direction == CW
[00:00:05.163,909] <inf> bldc_motor: MOT: PP check: OK!
[00:00:05.163,909] <inf> bldc_motor: MOT: Aligning zero electric angle (Dithering & Averaging)...
[00:00:06.211,975] <inf> bldc_motor: MOT: Zero elec. angle: 3.406
[00:00:06.212,005] <inf> foc_current_test:   [OK] FOC calibration complete!
[00:00:06.212,005] <inf> foc_current_test: 5. Initializing FOC current sense...
[00:00:06.212,097] <inf> current_sense: isense timer: PWM top=640 pre=0 UP -> period=640 ticks, sample@25 ticks (4%)
[00:00:06.212,127] <inf> current_sense: DPPI alloc: h_clear=30 h_sample=29 h_dis=28 h_en=27 grp=5 CHEN=0xd8000000
[00:00:06.212,127] <inf> current_sense: DPPI regs: PUB_PWMPERIODEND=0x8000001e SUB_TMRCLEAR=0x8000001e
[00:00:06.212,127] <inf> current_sense: DPPI regs: PUB_TMRCMP0=0x8000001d SUB_SAADCSMP=0x8000001d
[00:00:06.212,249] <inf> current_sense: TIMER2 cnt: t0=108 t1=451 diff=343 (h_clear OK if diff<640)
[00:00:06.212,402] <inf> current_sense: PWMPERIODEND: addr=0x50021118 fired=1 (1=sync OK; 0=PWM stopped/not started)
[00:00:06.212,402] <inf> current_sense: Periph addrs: NRF_PWM0=0x50021000 NRF_TIMER2=0x50011000 NRF_DPPIC=0x50017000
[00:00:06.217,468] <inf> current_sense: SAADC HW-timed ready: probe SOA=0.000 SOB=1.559 SOC=1.564 V (scans=7)
  [OK] FOC_CURRENT active!---> ANTICOGGING CALIBRATION START
---> ITERATION 1 / 5 ---
[00:00:07.061,614] <inf> current_sense: offsets calibrated: SOA=0.000 SOB=1.557 SOC=1.563 V (expect ~VREF/2, 681 synced scans)
[00:00:07.061,645] <inf> bldc_motor: current loop configured: bw=15 Hz  foc=5000 Hz  Kp=0.0094 V/A  Ki=527.8 V/(A*s) | LPF fc=50 Hz  Tf=3215 us  Ts=200 us
[00:00:07.871,582] <inf> bldc_motor: current-sense align: Ia(alpha-drive)=0.568 A -> gain_sign=+1
     Mean residual error = 0.0142 A (Max spike = 0.0350 A)
---> ITERATION 2 / 5 ---
     Mean residual error = 0.0145 A (Max spike = 0.0352 A)
---> ITERATION 3 / 5 ---
     Mean residual error = 0.0146 A (Max spike = 0.0368 A)
---> ITERATION 4 / 5 ---
     Mean residual error = 0.0149 A (Max spike = 0.0413 A)
---> ITERATION 5 / 5 ---
     Mean residual error = 0.0155 A (Max spike = 0.0460 A)
---> CALIBRATION COMPLETED ---
=== START LUT DUMP ===
Index,Current_A
0,0.009294
1,0.031134
2,0.047733
3,0.057897
4,0.061981
5,0.061123
6,0.056459
7,0.048784
8,0.038601
9,0.026301
10,0.012344
11,-0.002671
12,-0.018039
13,-0.033059
14,-0.047109
15,-0.059614
16,-0.069832
17,-0.076540
18,-0.077997
19,-0.072426
20,-0.058973
21,-0.038500
22,-0.013766
23,0.011261
24,0.032614
25,0.047584
26,0.055313
27,0.056543
28,0.052786
29,0.045555
30,0.036000
31,0.024941
32,0.013007
33,0.000748
34,-0.011346
35,-0.022854
36,-0.033394
37,-0.042552
38,-0.049749
39,-0.054131
40,-0.054583
41,-0.050027
42,-0.040003
43,-0.025222
44,-0.007710
45,0.009672
46,0.024062
47,0.033468
48,0.037239
49,0.036020
50,0.031182
51,0.024208
52,0.016296
53,0.008305
54,0.000817
55,-0.005804
56,-0.011364
57,-0.015820
58,-0.019216
59,-0.021609
60,-0.022976
61,-0.023149
62,-0.021788
63,-0.018508
64,-0.013128
65,-0.005951
66,0.002149
67,0.009879
68,0.015873
69,0.019099
70,0.019145
71,0.016268
72,0.011214
73,0.004978
74,-0.001352
75,-0.006669
76,-0.009935
77,-0.010367
78,-0.007682
79,-0.002271
80,0.004857
81,0.012336
82,0.018849
83,0.023457
84,0.025770
85,0.025865
86,0.024066
87,0.020711
88,0.016025
89,0.010113
90,0.003028
91,-0.005139
92,-0.014180
93,-0.023740
94,-0.033274
95,-0.042063
96,-0.049121
97,-0.053119
98,-0.052419
99,-0.045577
100,-0.032165
101,-0.013407
102,0.007907
103,0.028231
104,0.044371
105,0.054464
106,0.058331
107,0.057078
108,0.052196
109,0.044916
110,0.036014
111,0.026008
112,0.015335
113,0.004483
114,-0.006019
115,-0.015636
116,-0.023863
117,-0.030153
118,-0.033786
119,-0.033790
120,-0.029089
121,-0.019026
122,-0.003984
123,0.014285
124,0.032982
125,0.049024
126,0.059975
127,0.064691
128,0.063406
129,0.057243
130,0.047507
131,0.035268
132,0.021272
133,0.006117
134,-0.009628
135,-0.025334
136,-0.040354
137,-0.054085
138,-0.066058
139,-0.075895
140,-0.083135
141,-0.086967
142,-0.086070
143,-0.078867
144,-0.064136
145,-0.042156
146,-0.015253
147,0.012355
148,0.036092
149,0.052355
150,0.059695
151,0.058763
152,0.051746
153,0.041033
154,0.028475
155,0.015255
156,0.002306
157,-0.009488
158,-0.019341
159,-0.026718
160,-0.031368
161,-0.033259
162,-0.032331
163,-0.028301
164,-0.020658
165,-0.009082
166,0.005978
167,0.022954
168,0.039375
169,0.052543
170,0.060350
171,0.061882
172,0.057522
173,0.048534
174,0.036408
175,0.022411
176,0.007507
177,-0.007524
178,-0.021914
179,-0.034842
180,-0.045490
181,-0.053242
182,-0.057885
183,-0.059662
184,-0.059122
185,-0.056794
186,-0.052768
187,-0.046440
188,-0.036822
189,-0.023504
190,-0.007505
191,0.008681
192,0.021960
193,0.029691
194,0.030616
195,0.025173
196,0.015102
197,0.002424
198,-0.011213
199,-0.024465
200,-0.035722
201,-0.042936
202,-0.044203
203,-0.038663
204,-0.027074
205,-0.011620
206,0.004784
207,0.019598
208,0.031511
209,0.040621
210,0.047639
211,0.053014
212,0.056513
213,0.057424
214,0.055000
215,0.048878
216,0.039299
217,0.027000
218,0.012954
219,-0.001873
220,-0.016603
221,-0.030360
222,-0.042067
223,-0.050285
224,-0.053381
225,-0.050120
226,-0.040403
227,-0.025603
228,-0.008292
229,0.008573
230,0.022583
231,0.032539
232,0.038455
233,0.040977
234,0.040681
235,0.037747
236,0.032070
237,0.023585
238,0.012501
239,-0.000619
240,-0.014991
241,-0.029764
242,-0.044151
243,-0.057352
244,-0.068282
245,-0.075160
246,-0.075604
247,-0.067494
248,-0.050280
249,-0.025649
250,0.002732
251,0.030323
252,0.053097
253,0.068791
254,0.077202
255,0.079466
256,0.076980
257,0.070826
258,0.061728
259,0.050224
260,0.036827
261,0.022134
262,0.006834
263,-0.008361
264,-0.022807
265,-0.035932
266,-0.047108
267,-0.055466
268,-0.059760
269,-0.058658
270,-0.051341
271,-0.038177
272,-0.020893
273,-0.002300
274,0.014582
275,0.027346
276,0.034882
277,0.037293
278,0.035455
279,0.030418
280,0.023055
281,0.014033
282,0.003830
283,-0.007082
284,-0.018323
285,-0.029446
286,-0.040057
287,-0.049686
288,-0.057787
289,-0.063527
290,-0.065736
291,-0.063067
292,-0.054508
293,-0.040123
294,-0.021464
295,-0.001387
296,0.016793
297,0.030311
298,0.037760
299,0.039309
300,0.036270
301,0.030348
302,0.023011
303,0.015304
304,0.007896
305,0.001220
306,-0.004487
307,-0.009129
308,-0.012728
309,-0.015334
310,-0.016979
311,-0.017587
312,-0.016945
313,-0.014742
314,-0.010824
315,-0.005428
316,0.000681
317,0.006368
318,0.010407
319,0.011874
320,0.010402
321,0.006273
322,0.000260
323,-0.006593
324,-0.013089
325,-0.017951
326,-0.019946
327,-0.018236
328,-0.012704
329,-0.004130
330,0.005993
331,0.015885
332,0.024023
333,0.029500
334,0.032151
335,0.032332
336,0.030660
337,0.027674
338,0.023767
339,0.019117
340,0.013815
341,0.007868
342,0.001373
343,-0.005413
344,-0.011988
345,-0.017638
346,-0.021425
347,-0.022219
348,-0.018948
349,-0.011065
350,0.000963
351,0.015522
352,0.030205
353,0.042499
354,0.050530
355,0.053534
356,0.051822
357,0.046304
358,0.037954
359,0.027520
360,0.015533
361,0.002429
362,-0.011334
363,-0.025247
364,-0.038781
365,-0.051435
366,-0.062729
367,-0.072099
368,-0.078714
369,-0.081263
370,-0.077924
371,-0.066954
372,-0.047803
373,-0.022067
374,0.006572
375,0.033400
376,0.054185
377,0.066478
378,0.070178
379,0.067005
380,0.059278
381,0.048873
382,0.036985
383,0.024377
384,0.011707
385,-0.000373
386,-0.011210
387,-0.020242
388,-0.027045
389,-0.031298
390,-0.032617
391,-0.030404
392,-0.023963
393,-0.012946
394,0.002062
395,0.019301
396,0.036118
397,0.049691
398,0.057902
399,0.059894
400,0.056118
401,0.047805
402,0.036332
403,0.022816
404,0.008090
405,-0.007122
406,-0.022043
407,-0.035854
408,-0.047823
409,-0.057465
410,-0.064596
411,-0.069236
412,-0.071386
413,-0.070744
414,-0.066498
415,-0.057506
416,-0.043061
417,-0.023844
418,-0.002320
419,0.017889
420,0.033182
421,0.041147
422,0.041256
423,0.034799
424,0.024010
425,0.011022
426,-0.002557
427,-0.015425
428,-0.026243
429,-0.033714
430,-0.036938
431,-0.035810
432,-0.031110
433,-0.024211
434,-0.016505
435,-0.008788
436,-0.000954
437,0.007663
438,0.017461
439,0.027872
440,0.037346
441,0.043941
442,0.046076
443,0.043101
444,0.035468
445,0.024405
446,0.011282
447,-0.002789
448,-0.016912
449,-0.030033
450,-0.040679
451,-0.047072
452,-0.047726
453,-0.042177
454,-0.031391
455,-0.017512
456,-0.003109
457,0.009748
458,0.020110
459,0.028070
460,0.034100
461,0.038340
462,0.040370
463,0.039450
464,0.034992
465,0.026883
466,0.015588
467,0.001963
468,-0.013001
469,-0.028345
470,-0.043120
471,-0.056154
472,-0.065709
473,-0.069525
474,-0.065532
475,-0.052981
476,-0.033142
477,-0.009109
478,0.015105
479,0.035853
480,0.050969
481,0.060118
482,0.064148
483,0.064114
484,0.060693
485,0.054197
486,0.044810
487,0.032837
488,0.018839
489,0.003591
490,-0.012064
491,-0.027346
492,-0.041551
493,-0.053875
494,-0.063129
495,-0.067575
496,-0.065307
497,-0.055153
498,-0.037579
499,-0.014889
=== END LUT DUMP ===
DATA:27.324800,9.170617,-0.626844,45.700314,-66.605042
DATA:7.355425,8.444540,-0.559286,45.904728,-66.804573
DATA:-0.109175,5.023054,-0.218039,45.401016,-66.904335
DATA:-0.381240,2.861337,0.002815,45.306133,-66.609924
DATA:-0.011592,1.712165,0.125724,45.306133,-66.904335
DATA:0.067873,1.054449,0.201295,45.405899,-66.804573
DATA:0.064907,0.658632,0.251730,45.301250,-66.804573
DATA:-0.002738,0.394084,0.289698,45.101719,-66.500389
DATA:0.126086,0.286885,0.312303,45.600548,-66.809456
DATA:0.154603,0.233972,0.329639,45.909611,-66.609924
DATA:0.047485,0.159377,0.349272,45.804962,-66.405510
DATA:0.202960,0.176811,0.359758,46.004494,-66.400627
DATA:0.182607,0.179129,0.371736,46.208908,-66.300858
DATA:0.189823,0.183407,0.383512,46.204025,-66.101326
DATA:0.031704,0.122725,0.401840,46.603085,-66.201096
DATA:0.044635,0.091489,0.417315,46.308670,-65.802032
DATA:-0.016715,0.048208,0.434070,46.503319,-65.806915
DATA:0.042496,0.045923,0.446770,46.807499,-65.802032
DATA:0.019846,0.035492,0.460298,46.503319,-65.901794
DATA:0.021684,0.029969,0.473351,46.807499,-65.702263
DATA:-0.073175,-0.011289,0.490024,47.101913,-65.702263
DATA:0.224479,0.083018,0.493087,46.907265,-65.707146
DATA:0.189848,0.125750,0.501171,47.007030,-65.507614
DATA:0.018081,0.082683,0.517836,47.002148,-65.407852
DATA:0.066737,0.076304,0.530881,47.301445,-65.602501
DATA:0.253859,0.147326,0.536121,47.401211,-65.303200
DATA:0.090489,0.124591,0.550688,47.406094,-65.407852
DATA:0.167445,0.141733,0.561274,47.505859,-65.308083
DATA:0.116646,0.131698,0.574570,47.705391,-64.909027
DATA:0.111885,0.123773,0.587673,47.605625,-64.909027
DATA:0.013418,0.079631,0.604450,47.900040,-64.904144
DATA:0.027323,0.058708,0.618970,47.900040,-64.904144
DATA:0.006829,0.037956,0.633514,48.109337,-64.904144
DATA:0.055541,0.044990,0.645294,48.204220,-64.709496
DATA:0.021861,0.035739,0.658704,47.900040,-64.709496
DATA:0.111544,0.066061,0.668136,48.308868,-64.709496
DATA:0.158742,0.103133,0.676826,48.503517,-64.604843
DATA:0.048464,0.081266,0.691394,48.308868,-64.704613
DATA:0.025449,0.058939,0.706053,48.204220,-64.505081
DATA:0.026202,0.045844,0.719823,48.603283,-64.505081
DATA:0.034509,0.041310,0.732756,48.408634,-64.505081
DATA:0.041734,0.041480,0.745222,48.503517,-64.509964
DATA:0.025168,0.034955,0.758364,48.907459,-64.405312
DATA:0.051833,0.041706,0.770178,48.608166,-64.305550
DATA:0.063228,0.050315,0.781791,48.802811,-64.405312
DATA:0.051122,0.050638,0.794224,48.907459,-64.106018
DATA:0.106156,0.072845,0.804446,49.007225,-64.200897
DATA:0.046950,0.062487,0.817912,49.201874,-64.006248
DATA:0.053023,0.058702,0.830736,49.106991,-64.006248
DATA:0.127822,0.086350,0.840392,49.301640,-64.001366
DATA:0.053445,0.073188,0.854114,49.201874,-63.801834
DATA:0.075419,0.074080,0.866444,49.401405,-63.906483
DATA:0.040722,0.060737,0.880209,49.700703,-63.806717
DATA:0.086861,0.071187,0.891599,49.501171,-63.806717
DATA:0.084668,0.076579,0.903478,49.506054,-63.801834
DATA:0.053574,0.067377,0.916820,49.700703,-64.001366
DATA:-0.050941,0.020050,0.934031,49.900234,-63.507423
DATA:0.089269,0.047738,0.943761,49.605820,-63.407658
DATA:0.058775,0.052153,0.955785,49.800468,-63.602306
DATA:0.035762,0.045597,0.968909,49.900234,-63.502541
DATA:0.090538,0.063573,0.979568,49.800468,-63.607189
DATA:0.129929,0.090116,0.989326,49.900234,-63.402775
DATA:0.119279,0.101781,1.000534,50.004883,-63.507423
DATA:0.133730,0.114561,1.011606,50.304180,-63.402775
DATA:0.158019,0.131944,1.022187,50.503712,-62.908829
DATA:0.036094,0.093604,1.038361,50.503712,-62.903946
DATA:0.009384,0.059916,1.054142,50.508595,-62.903946
DATA:0.424646,0.205808,1.051853,51.301834,-62.500000
DATA:0.565635,0.349739,1.049471,51.501366,-62.001171
DATA:0.917023,0.576653,1.038419,52.409023,-61.502342
DATA:1.285687,0.860266,1.021187,53.606213,-60.105625
DATA:2.084410,1.349924,0.982577,55.900822,-58.309845
DATA:2.702744,1.891052,0.937789,58.205196,-55.307110
DATA:3.331172,2.467100,0.888392,61.203049,-52.304375
DATA:4.741376,3.376811,0.804143,65.103668,-47.500977
DATA:3.794410,3.543850,0.793085,66.904335,-44.009178
DATA:2.967945,3.313488,0.821829,69.308479,-41.106213
  */