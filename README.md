# Embreadded Symptoms - Firmware for Synth Module

This repo contains our code for our the second coursework of the Embedded Systems module (ELEC 96018).

## File Structure
The folder [*Code/synth*](Code/synth) contains our code for the synth module. It has these files:
- [*LUTs.h*](Code/synth/LUTs.h) - Header file containing lookup tables used in the main code. These help with generation of waveforms and effects.
- [*knob.h*](Code/synth/knob.h) - Defines a knob class that is used to encapsulate the properties and functionality of knobs e.g. the update of values in response to received waveforms.
- [*synth.ino*](Code/synth/synth.ino) - The main code for the firmware of the synth. Contains implementation of threads and interrupt routines responsible for the different tasks.
- [*synth.bin*](Code/synth/synth.bin) - An executable binary compiled from the above code files. Same as our submission for the coursework. 

## How to Run
In order to upload the synth implementation to the board, open Arduino, Sketch>Upload.

Alternatively use the compiled **binary found in Code/synth/**

## Link to Advanced Features video
https://youtu.be/3AHNhz73ujM

## Description of Our Implementation
We have implemented the functional functional features asked for in the Coursework specification:
-  *Serial input/output of messages*, according to the desired protocol.
-  *Playing sawtooth waves* of correct note frequency without perceptible delay from note press.  
-  *Volume control* with exactly 8 increments
-  *OLED display* of volume and notes played.

Additionally, the following advanced features are implemented
- *Sine, square, exponential and triangle* waveforms.
- *Smoothing filter* (can be applied on any waveforms)
- *Polyphony* for sawtooth and triangle waveforms. **Note**: This is implemented for concurrent key presses, but there is no support for concurrent processing of input serial messages - this is done sequentially (one-by-one) through the queue buffer.
- *Vibrato* for sawtooth and triangle waveforms.
- *Echo* for sine waveform.
- *Tremolo* for all waveforms.
- *Superimposition of features* configurable from different knobs (see next section on **Using the Synth**).
- Presence/absence of the effects are *indicated on the OLED display* with imperceptible lag between turning of knob and change of the display.

All of this is demonstrated in the video linked in the above section.

## Using the Synth

For visual examples of using the synth, refer to the video linked above.

### Knobs

The knobs are used to adjust different aspects of the sound played. The knob values are adjusted by rotating them clockwise or anticlockwise.

- *Knob 0*: Choose Filter: **0** - No filtering, **1** - Smoothing filter.
- *Knob 1*: Change Effect: **0** - No effect, **1** - Vibrato/Echo (if available), **2** - Tremolo.
- *Knob 2*: Change Waveform: **0** - Saw, **1** - Sine, **2** - Square, **3** - Exponential, **4** - Triangle.
- *Knob 3*: Change volume: 0 to 16 in increments of 2

### Joystick
The joystick currently does not have any functionality assigned to it. It was found to be too sensitive for any practical usage that we could implement. A possible future extension to our software could be to create a functionality that is controlled by joystick inputs. However for now, moving the joystick will have no effect.

### Default Configuration

This configuration corresponds to the functional (non-advanced) requirements in the coursework specification. It plays a sawtooth waveform without any advanced features.

The knob configuration must be set so that the following display is seen on the OLED:

<img src="https://user-images.githubusercontent.com/56508438/112630873-1c567500-8e2e-11eb-990e-7490e18b0ba9.jpg" width="500" height="300">


## Superimposing Effects
The effects created by the different knobs (see section on **Knobs**) can be combined with each other in any manner without interference. An example can be seen at the end of the linked video.

