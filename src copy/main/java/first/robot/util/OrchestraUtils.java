package first.robot.util;

import com.ctre.phoenix6.Orchestra;

public class OrchestraUtils {
  public static String[] allSongs = {
    "_TEST1.chrp",
    "_TEST2.chrp",
    "_TEST3.chrp",
    "1148MainTheme.chrp",
    "AtDoomsGate.chrp",
    "AttackOfTheKillerQueen.chrp",
    "BabyBlue.chrp",
    "BadApple.chrp",
    "Bonetrousle.chrp",
    "Caramelldansen.chrp",
    "CaveStory.chrp",
    "DaisyBell.chrp",
    "DancingQueen.chrp",
    "EPP.chrp",
    "EverybodyWantsToRuleTheWorld.chrp",
    "HelloWorld.chrp",
    "HeyYa.chrp",
    "HotelCalifornia.chrp",
    "JustTheTwoOfUs.chrp",
    "M.chrp",
    "MonkeyIsland.chrp",
    "MrBlueSky.chrp",
    "MyHeartWillGoOn.chrp",
    "NoOnesAroundToHelp.chrp",
    "OneWingedAngel.chrp",
    "RunningFromEvil.chrp",
    "RunningFromEvilAlt.chrp",
    "RushE.chrp",
    "ScatmansWorld.chrp",
    "ShootingStars.chrp",
    "SomewhereOverTheRainbow.chrp",
    "StillAlive.chrp",
    "TakeOnMe.chrp",
    "the_man_.chrp",
    "TheFinalCountdown.mid.chrp",
    "TheWorldRevolving.chrp",
    "WantYouGone.chrp",
    "WhatAWonderfulWorld.chrp"
  };
  public static int songSelected = 0;
  public static int songPlaying = -1;
  public static boolean isPlaying = false;
  public static Orchestra orchestra = new Orchestra();
}
