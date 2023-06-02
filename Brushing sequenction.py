import rp2
from machine import Pin, PWM
from utime import sleep
import machine
import utime
import time,array
led_count = 16 
PIN_NUM = 13 
brightness = 0.3

@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT,
             autopull=True, pull_thresh=24) 

def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()


state_mach = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=Pin(PIN_NUM))

state_mach.active(1)

pixel_array = array.array("I", [0 for _ in range(led_count)])

def update_pix(brightness_input=brightness): 
    dimmer_array = array.array("I", [0 for _ in range(led_count)])
    for ii,cc in enumerate(pixel_array):
        r = int(((cc >> 8) & 0xFF) * brightness_input)
        g = int(((cc >> 16) & 0xFF) * brightness_input)
        b = int((cc & 0xFF) * brightness_input)
        dimmer_array[ii] = (g<<16) + (r<<8) + b 
    state_mach.put(dimmer_array, 8) 
    time.sleep_ms(10)

def set_24bit(ii, color): 
    pixel_array[ii] = (color[1]<<16) + (color[0]<<8) + color[2] 

def brushing(color, blank): 
 for i in [0,4,8,12]:
                set_24bit(i,color)
                update_pix()
                set_24bit(i+1,color)
                update_pix()
                set_24bit(i+2,color)
                update_pix()
                set_24bit(i+3,color)
                update_pix()
                time.sleep(30.0)
                buzzer.freq(500)
                buzzer.duty_u16(1000)
                sleep(0.05)
                set_24bit(i,blank)
                update_pix()
                set_24bit(i+1,blank)
                update_pix()
                set_24bit(i+2,blank)
                update_pix()
                set_24bit(i+3,blank)
                update_pix()
                buzzer.duty_u16(0)
def blanking(color):
     for i in range(len(pixel_array)):
           set_24bit(i,color)
           update_pix()
           
def ending(color, blank):
        cycles = 1 
        buzzer.duty_u16(1000)
        time.sleep(1.0)
        for ii in range(int(cycles*len(pixel_array))+1):
            for jj in range(len(pixel_array)):
                if jj==int(ii%led_count): 
                    set_24bit(jj,color) 
                else:
                    set_24bit(jj,blank) 
            update_pix() 
            time.sleep(0.05) 
        buzzer.duty_u16(0)
        
endColor = 0            
color = (255,0,255)
blank = (0,0,0)
button = Pin(14, Pin.IN, Pin.PULL_DOWN)
buzzer = PWM(Pin(15))
buzzer.duty_u16(0)

Battery_val = machine.ADC(26)

while True:
    reading = Battery_val.read_u16()     
    print("ADC: ",reading)
    utime.sleep(0.2)
    
    if button.value():
        reading = Battery_val.read_u16()
        if reading <= 47000:
            endColor = (255,0,0)
        if reading >= 47000 and reading <= 52000:
            endColor = (255,100,0)
        if reading > 52000:
            endColor = (0,255,0)
        brushing(color, blank) 
        ending(endColor, blank)
        blanking((0 ,0,0))                
       
      