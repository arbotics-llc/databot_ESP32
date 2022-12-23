void creatTask()
{
  xTaskCreatePinnedToCore(
    Task1code, /* Function to implement the task */
    "buzz", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &buzz,  /* Task handle. */
    0); /* Core where the task should run */

  vTaskSuspend(buzz);

  xTaskCreatePinnedToCore(
    Task2code, /* Function to implement the task */
    "led", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    1,  /* Priority of the task */
    &led,  /* Task handle. */
    0); /* Core where the task should run */

  vTaskSuspend(led);

}


void Task1code( void * parameter ) {
  //  Dprint("Buzz is running on core ");
  //  Dprintln(xPortGetCoreID());
  for (;;) {

    // iterate over the notes of the melody.
    // Remember, the array is twice the number of notes (notes + durations)
    for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

      // calculates the duration of each note
      divider = melody[thisNote + 1];
      if (divider > 0) {
        // regular note, just proceed
        noteDuration = (wholenote) / divider;
      } else if (divider < 0) {
        // dotted notes are represented with negative durations!!
        noteDuration = (wholenote) / abs(divider);
        noteDuration *= 1.5; // increases the duration in half for dotted notes
      }

      // we only play the note for 90% of the duration, leaving 10% as a pause
      //    tone(buzzer, melody[thisNote], noteDuration * 0.9);
      tone(BUZZER_PIN, melody[thisNote], noteDuration * 0.9, BUZZER_CHANNEL);
      // Wait for the specief duration before playing the next note.
      delay(noteDuration);

      // stop the waveform generation before the next note.
      //    noTone(buzzer);
      noTone(BUZZER_PIN, BUZZER_CHANNEL);
    }
  }
  //  delay(3000);
}

void Task2code( void * parameter ) {
  for (;;) {
    //    RGBled.setPixelColor(0, RGBled.Color(random(0, 255), random(0, 255), random(0, 255)));
    //    RGBled.setPixelColor(1, RGBled.Color(random(0, 255), random(0, 255), random(0, 255)));
    //    RGBled.setPixelColor(2, RGBled.Color(random(0, 255), random(0, 255), random(0, 255)));
    RGBled.setPixelColor(0, RGBled.Color(255, 0, 0));
    RGBled.setPixelColor(1, RGBled.Color(0, 0, 255));
    RGBled.setPixelColor(2, RGBled.Color(0, 0, 255));
    RGBled.show();
    delay(100);

    RGBled.setPixelColor(0, RGBled.Color(0, 0, 255));
    RGBled.setPixelColor(1, RGBled.Color(255, 0, 0));
    RGBled.setPixelColor(2, RGBled.Color(0, 0, 255));
    RGBled.show();
    delay(100);

    RGBled.setPixelColor(0, RGBled.Color(0, 0, 255));
    RGBled.setPixelColor(1, RGBled.Color(0, 0, 255));
    RGBled.setPixelColor(2, RGBled.Color(255, 0, 0));
    RGBled.show();
    delay(100);
  }
}
