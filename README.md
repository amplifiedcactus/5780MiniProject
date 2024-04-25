# 5780MiniProject

Group members:

Stephen Ehlert
Ajayvarman Mallapillai
Sriaishwarya Sivagurunathan


PROJECT TITLE:

MIDI (Musical Instrument Digital Interface) controller and sequencer

PURPOSE

The purpose of this project is to create something interesting and fun using the skills we learned throughout the semester in ECE 5780/6780. It will be submitted as the "mini-project" assignment for the class.

DESCRIPTION:

We have built a MIDI controller and sequencer using the STM32F0 microcontroller discovery board. It includes 6 buttons, 1 potentiometer, and a MIDI output (through a DIN connector). The MIDI controller has two modes: play and pause mode. In pause mode, pressing buttons in a row of 5 buttons on the device results in MIDI note commands being output on the MIDI interface, with a velocity that depends on the potentiometer value. The note and velocity value also get added to an array or sequence of 8 notes stored in memory. Pressing the 6th button will switch the device to play mode, where the sequence is played by sending note commands to the MIDI output. In play mode, the frequency or tempo that the note commands are sent depends on the potentiometer value. Turning the potentiometer counter-clockwise will increase the tempo, and turning it clockwise will increase the tempo.

HARDWARE SETUP:

We are using 5 external pushbuttons, 1 pushbutton that is provided on the discovery board, a potentiometer, a 74AHCT125 for converting 3.3V signals to 5V, 2 220 Ohm resistors, and a MIDI DIN connector. See wiring diagram for connections to discovery board.

We have tested the controller/sequencer using a COTS (commercial off the shelf) USB to MIDI interface, a laptop, and a online MIDI keyboard tester (virtualpiano.eu). See videos below.

WIRING DIAGRAM:

SOFTWARE DESCRIPTION:

VIDEOS:

REFERENCE:

http://www.shclemen.com/download/The%20Complete%20MIDI1.0%20Detailed%20Spec.pdf
https://learn.sparkfun.com/tutorials/midi-tutorial/hardware--electronic-implementation


