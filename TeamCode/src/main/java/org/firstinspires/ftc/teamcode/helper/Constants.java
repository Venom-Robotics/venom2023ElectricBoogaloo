package org.firstinspires.ftc.teamcode.helper;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class Constants {
    public static final class Drivetrain {
//        public final static double WHEEL_DIAMETER = 3.7795275590551185; // 96 mm in Inches (goBILDA Mecanums)
        public final static double WHEEL_DIAMETER = 2.95275590552; // 75 mm in Inches (REV Mecanums)
        public final static double COUNTS_PER_MOTOR_REVOLUTION = 530.051282051282; // Ticks (UltraPlanetary 4:1 -> 5:1)
        public final static double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        public final static double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REVOLUTION / WHEEL_CIRCUMFERENCE;

        public static final double MOVEMENT_TOLERANCE = 30.0; // Ticks
        public static final double TURNING_TOLERANCE = 1.0; // Angle

        public static final double P_TURN = 0.02;

        public static final class Calculations {
            public static double DPAD_SPEED = 0.7;
            public static double TOTAL_MULTIPLIER = 1.0;
        }

    }

    public final static class Presets {
        @Config
        public final static class Turret {
            public static int LEFT = 210;
            public static int RIGHT = -223;
            public static int FORWARD = 0;
            public static int BACK = 362;
        }
        @Config
        public static class Arm {
            public static int GROUND = 0;
            public static int LOW = 0;
            public static int MED = 0;
            public static int HIGH = 0;
        }
        @Config
        public static class Lift {
            public static int GROUND = 210;
            public static int LOW = -223;
            public static int MED = 0;
            public static int HIGH = 362;
        }
    }
}
