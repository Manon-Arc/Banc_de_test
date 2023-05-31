import tm1637
from machine import Pin
from time import sleep

tm= tm1637.TM1637(dio=Pin(21), clk=Pin(22))

while True :   
    tm.show('abcd')
    sleep(1)
    tm.number(1234)
    sleep(1)
    tm.numbers(12,34)
    sleep(1)
    tm.scroll('Hello World')