This ESPIDF project written in 'C' & 'C++', uses Waveshare's 7" display/ESP32s3.  
As the project name suggests, it accepts an analog audio signal (~2Vp-p) input, via the Waveshare's Analog senor jack.
And after DSP, can output the 'processed' data in two formats:  
A. as a USB UAC 16Khz single chanel audio.  
B. as an analog (PDM) signal (~1Vp-p) via Waveshare's UART2 RX lead.  
Note: Only one output mode is available at a time, and is NOT switchable 'on the fly'. 
But can be changed, using 2nd Display button(I2S/PDM), followed by a board 'reset'.  
Currently there are 4 DSP modes (which are switchable 'on the fly'):
1. Straight pass though.
2. IIR filter (~300hz BW, centered @ 750Hz).
3. NSNET2 voice noise filter.
4. A modified Wiener tone filter (range ~450 to 900 Hz).  

The project's original intent is to showcase/examine how these DSP filtering techniques can be used in a ham radio environment.  
Primarilly to recover noisy CW signals, and in the case of the NSNET2 filter, SSB vioce audio.  
While the ESPIDF offers a wide range of libraries like these, their use is often not well documented.  
So for those interested in exploring these methods, (plus some LVGL features), this may be of value.    
