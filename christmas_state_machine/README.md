# Study State Machine

This package manages the state on the robot throughout the course of the interruptibility study.

## Generating sounds

Make sure to install the sound packs from the instructions at [this URL](https://ubuntuforums.org/showthread.php?t=751169). Once done, create a text file with the words that you want to speak and generate the utterance with the following command:

```
text2wave -eval '(voice_cmu_us_rms_arctic_clunits)' <text filename> -o <wav filename>
```

You can test the output with the following command: `aplay <wav filename>`
