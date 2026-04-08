## Position:
mv:x:-10  //relative move of -10 units on the x-axis
mv:y:15   //relative move of 15 units on the y-axis
mv:z:5    //relative move of 5 units on the z-axis
mv:e:20   //relative move of 20 units on the extruder axis

## Temperature Change:
tp:e:-5  //absolute decrease of 5 degrees in temperature
tp:b:20  //absolute temperature set to 20 degrees

## Set Printing Tool:
t:0    //select tool 0 for printing
t:1    //select tool 1 for printing
..     //additional tool selections as needed

## Printer Light Control:
l:on   //turn on the printer light
l:off  //turn off the printer light

## Z-offset Adjustment:
offset:+0.1  //increase Z-offset by 0.1 units

## Change Print Sheet:
s:custom0    //switch to print sheet 1
s:custom1    //switch to print sheet 2
s:custom2    //switch to print sheet 3
..            //additional print sheet selections as needed

## Keys: // TO DO
key:custom1  //custom key for specific function 1
key:custom2  //custom key for specific function 2
..           //additional custom keys as needed

//custom keys and sheets definition will be handled in RPI via Flask server