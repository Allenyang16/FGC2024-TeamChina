# FGC Team China Codes
This repository is used for FIRST Global Challenge 2024 [Feeding the Future](https://www.youtube.com/watch?v=VoT_eVi7vQ0)

To explore this repository, go to [src directory](https://github.com/Allenyang16/FGC2024-TeamChina/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode)

Our hardwares are instantiated in the subsystems, and commands contain the actions we need to use during the whole match.

For more information, go to our [Chief Delphi](https://www.chiefdelphi.com/t/fgc-2024-team-china-engineering-book-cad-code-release/472296) to ask more questions!

1. What **IDE** are you using?

   We use [Android Studio](https://ftc-docs.firstinspires.org/en/latest/programming_resources/android_studio_java/Android-Studio-Tutorial.html) and Java to code our robots. And you need to connect your device with Control Hub to upload your codes.

2. What **third-party libraries** are you using?

   In this project, we utilized
   
   + [FTCLib](https://docs.ftclib.org/ftclib), which users can use their FRC-like interfaces.
   + [FTC Dashboard](https://acmerobotics.github.io/ftc-dashboard/), which users can easily output desired values and change the related parameters while debugging their codes or hardwares.

3. What advanced **controls** are you using?

   For our FGC 2024 robot, we used [PID controller](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html) to control the position of our intake/claw, a mechanism to help drivers grab **FOOD** balls. You can check this out in the [*Intake.java*](https://github.com/Allenyang16/FGC2024-TeamChina/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Intake.java).

4. What other **features** you have?

   We used *lots of* state machines, which are the [enum classes](https://www.baeldung.com/a-guide-to-java-enums) in our subsystems. With the use of it, we can simplify our codes by only considering what actions the motors or servos need to do corresponding to the current state. When buttons are triggered, we need to only change state of the subsystem without considering too much about basic operations on hardwares.
     

