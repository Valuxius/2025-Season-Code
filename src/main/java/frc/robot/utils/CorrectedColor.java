// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class CorrectedColor extends Color {
    private String m_name;

    /**
   * Constructs a CorrectedColor from doubles.
   *
   * @param red Red value (0-1)
   * @param green Green value (0-1)
   * @param blue Blue value (0-1)
   */
  public CorrectedColor(double red, double green, double blue) {
    super(roundAndClamp(green), roundAndClamp(red), roundAndClamp(blue));
  }

  /**
   * Constructs a Color from doubles.
   *
   * @param red Red value (0-1)
   * @param green Green value (0-1)
   * @param blue Blue value (0-1)
   */
  private CorrectedColor(double red, double green, double blue, String name) {
    super(roundAndClamp(green), roundAndClamp(red), roundAndClamp(blue));
    this.m_name = name;
  }

  private static double roundAndClamp(double value) {
    return MathUtil.clamp(Math.ceil(value * (1 << 12)) / (1 << 12), 0.0, 1.0);
  }

  /*
   * FIRST Colors
   */

  /** 0x1560BD. */
  public static final CorrectedColor kDenim = new CorrectedColor(0.0823529412, 0.376470589, 0.7411764706, "kDenim");

  /** 0x0066B3. */
  public static final CorrectedColor kFirstBlue = new CorrectedColor(0.0, 0.4, 0.7019607844, "kFirstBlue");

  /** 0xED1C24. */
  public static final CorrectedColor kFirstRed =
      new CorrectedColor(0.9294117648, 0.1098039216, 0.1411764706, "kFirstRed");

  /*
   * Standard CorrectedColors
   */

  /** 0xF0F8FF. */
  public static final CorrectedColor kAliceBlue = new CorrectedColor(0.9411765f, 0.972549f, 1.0f, "kAliceBlue");

  /** 0xFAEBD7. */
  public static final CorrectedColor kAntiqueWhite =
      new CorrectedColor(0.98039216f, 0.92156863f, 0.84313726f, "kAntiqueWhite");

  /** 0x00FFFF. */
  public static final CorrectedColor kAqua = new CorrectedColor(0.0f, 1.0f, 1.0f, "kAqua");

  /** 0x7FFFD4. */
  public static final CorrectedColor kAquamarine = new CorrectedColor(0.49803922f, 1.0f, 0.83137256f, "kAquamarine");

  /** 0xF0FFFF. */
  public static final CorrectedColor kAzure = new CorrectedColor(0.9411765f, 1.0f, 1.0f, "kAzure");

  /** 0xF5F5DC. */
  public static final CorrectedColor kBeige = new CorrectedColor(0.9607843f, 0.9607843f, 0.8627451f, "kBeige");

  /** 0xFFE4C4. */
  public static final CorrectedColor kBisque = new CorrectedColor(1.0f, 0.89411765f, 0.76862746f, "kBisque");

  /** 0x000000. */
  public static final CorrectedColor kBlack = new CorrectedColor(0.0f, 0.0f, 0.0f, "kBlack");

  /** 0xFFEBCD. */
  public static final CorrectedColor kBlanchedAlmond =
      new CorrectedColor(1.0f, 0.92156863f, 0.8039216f, "kBlanchedAlmond");

  /** 0x0000FF. */
  public static final CorrectedColor kBlue = new CorrectedColor(0.0f, 0.0f, 1.0f, "kBlue");

  /** 0x8A2BE2. */
  public static final CorrectedColor kBlueViolet =
      new CorrectedColor(0.5411765f, 0.16862746f, 0.8862745f, "kBlueViolet");

  /** 0xA52A2A. */
  public static final CorrectedColor kBrown = new CorrectedColor(0.64705884f, 0.16470589f, 0.16470589f, "kBrown");

  /** 0xDEB887. */
  public static final CorrectedColor kBurlywood =
      new CorrectedColor(0.87058824f, 0.72156864f, 0.5294118f, "kBurlywood");

  /** 0x5F9EA0. */
  public static final CorrectedColor kCadetBlue =
      new CorrectedColor(0.37254903f, 0.61960787f, 0.627451f, "kCadetBlue");

  /** 0x7FFF00. */
  public static final CorrectedColor kChartreuse = new CorrectedColor(0.49803922f, 1.0f, 0.0f, "kChartreuse");

  /** 0xD2691E. */
  public static final CorrectedColor kChocolate =
      new CorrectedColor(0.8235294f, 0.4117647f, 0.11764706f, "kChocolate");

  /** 0xFF7F50. */
  public static final CorrectedColor kCoral = new CorrectedColor(1.0f, 0.49803922f, 0.3137255f, "kCoral");

  /** 0x6495ED. */
  public static final CorrectedColor kCornflowerBlue =
      new CorrectedColor(0.39215687f, 0.58431375f, 0.92941177f, "kCornflowerBlue");

  /** 0xFFF8DC. */
  public static final CorrectedColor kCornsilk = new CorrectedColor(1.0f, 0.972549f, 0.8627451f, "kCornsilk");

  /** 0xDC143C. */
  public static final CorrectedColor kCrimson = new CorrectedColor(0.8627451f, 0.078431375f, 0.23529412f, "kCrimson");

  /** 0x00FFFF. */
  public static final CorrectedColor kCyan = new CorrectedColor(0.0f, 1.0f, 1.0f, "kCyan");

  /** 0x00008B. */
  public static final CorrectedColor kDarkBlue = new CorrectedColor(0.0f, 0.0f, 0.54509807f, "kDarkBlue");

  /** 0x008B8B. */
  public static final CorrectedColor kDarkCyan = new CorrectedColor(0.0f, 0.54509807f, 0.54509807f, "kDarkCyan");

  /** 0xB8860B. */
  public static final CorrectedColor kDarkGoldenrod =
      new CorrectedColor(0.72156864f, 0.5254902f, 0.043137256f, "kDarkGoldenrod");

  /** 0xA9A9A9. */
  public static final CorrectedColor kDarkGray = new CorrectedColor(0.6627451f, 0.6627451f, 0.6627451f, "kDarkGray");

  /** 0x006400. */
  public static final CorrectedColor kDarkGreen = new CorrectedColor(0.0f, 0.39215687f, 0.0f, "kDarkGreen");

  /** 0xBDB76B. */
  public static final CorrectedColor kDarkKhaki =
      new CorrectedColor(0.7411765f, 0.7176471f, 0.41960785f, "kDarkKhaki");

  /** 0x8B008B. */
  public static final CorrectedColor kDarkMagenta =
      new CorrectedColor(0.54509807f, 0.0f, 0.54509807f, "kDarkMagenta");

  /** 0x556B2F. */
  public static final CorrectedColor kDarkOliveGreen =
      new CorrectedColor(0.33333334f, 0.41960785f, 0.18431373f, "kDarkOliveGreen");

  /** 0xFF8C00. */
  public static final CorrectedColor kDarkOrange = new CorrectedColor(1.0f, 0.54901963f, 0.0f, "kDarkOrange");

  /** 0x9932CC. */
  public static final CorrectedColor kDarkOrchid = new CorrectedColor(0.6f, 0.19607843f, 0.8f, "kDarkOrchid");

  /** 0x8B0000. */
  public static final CorrectedColor kDarkRed = new CorrectedColor(0.54509807f, 0.0f, 0.0f, "kDarkRed");

  /** 0xE9967A. */
  public static final CorrectedColor kDarkSalmon =
      new CorrectedColor(0.9137255f, 0.5882353f, 0.47843137f, "kDarkSalmon");

  /** 0x8FBC8F. */
  public static final CorrectedColor kDarkSeaGreen =
      new CorrectedColor(0.56078434f, 0.7372549f, 0.56078434f, "kDarkSeaGreen");

  /** 0x483D8B. */
  public static final CorrectedColor kDarkSlateBlue =
      new CorrectedColor(0.28235295f, 0.23921569f, 0.54509807f, "kDarkSlateBlue");

  /** 0x2F4F4F. */
  public static final CorrectedColor kDarkSlateGray =
      new CorrectedColor(0.18431373f, 0.30980393f, 0.30980393f, "kDarkSlateGray");

  /** 0x00CED1. */
  public static final CorrectedColor kDarkTurquoise =
      new CorrectedColor(0.0f, 0.80784315f, 0.81960785f, "kDarkTurquoise");

  /** 0x9400D3. */
  public static final CorrectedColor kDarkViolet = new CorrectedColor(0.5803922f, 0.0f, 0.827451f, "kDarkViolet");

  /** 0xFF1493. */
  public static final CorrectedColor kDeepPink = new CorrectedColor(1.0f, 0.078431375f, 0.5764706f, "kDeepPink");

  /** 0x00BFFF. */
  public static final CorrectedColor kDeepSkyBlue = new CorrectedColor(0.0f, 0.7490196f, 1.0f, "kDeepSkyBlue");

  /** 0x696969. */
  public static final CorrectedColor kDimGray = new CorrectedColor(0.4117647f, 0.4117647f, 0.4117647f, "kDimGray");

  /** 0x1E90FF. */
  public static final CorrectedColor kDodgerBlue = new CorrectedColor(0.11764706f, 0.5647059f, 1.0f, "kDodgerBlue");

  /** 0xB22222. */
  public static final CorrectedColor kFirebrick =
      new CorrectedColor(0.69803923f, 0.13333334f, 0.13333334f, "kFirebrick");

  /** 0xFFFAF0. */
  public static final CorrectedColor kFloralWhite = new CorrectedColor(1.0f, 0.98039216f, 0.9411765f, "kFloralWhite");

  /** 0x228B22. */
  public static final CorrectedColor kForestGreen =
      new CorrectedColor(0.13333334f, 0.54509807f, 0.13333334f, "kForestGreen");

  /** 0xFF00FF. */
  public static final CorrectedColor kFuchsia = new CorrectedColor(1.0f, 0.0f, 1.0f, "kFuchsia");

  /** 0xDCDCDC. */
  public static final CorrectedColor kGainsboro =
      new CorrectedColor(0.8627451f, 0.8627451f, 0.8627451f, "kGainsboro");

  /** 0xF8F8FF. */
  public static final CorrectedColor kGhostWhite = new CorrectedColor(0.972549f, 0.972549f, 1.0f, "kGhostWhite");

  /** 0xFFD700. */
  public static final CorrectedColor kGold = new CorrectedColor(1.0f, 0.84313726f, 0.0f, "kGold");

  /** 0xDAA520. */
  public static final CorrectedColor kGoldenrod =
      new CorrectedColor(0.85490197f, 0.64705884f, 0.1254902f, "kGoldenrod");

  /** 0x808080. */
  public static final CorrectedColor kGray = new CorrectedColor(0.5019608f, 0.5019608f, 0.5019608f, "kGray");

  /** 0x008000. */
  public static final CorrectedColor kGreen = new CorrectedColor(0.0f, 0.5019608f, 0.0f, "kGreen");

  /** 0xADFF2F. */
  public static final CorrectedColor kGreenYellow = new CorrectedColor(0.6784314f, 1.0f, 0.18431373f, "kGreenYellow");

  /** 0xF0FFF0. */
  public static final CorrectedColor kHoneydew = new CorrectedColor(0.9411765f, 1.0f, 0.9411765f, "kHoneydew");

  /** 0xFF69B4. */
  public static final CorrectedColor kHotPink = new CorrectedColor(1.0f, 0.4117647f, 0.7058824f, "kHotPink");

  /** 0xCD5C5C. */
  public static final CorrectedColor kIndianRed =
      new CorrectedColor(0.8039216f, 0.36078432f, 0.36078432f, "kIndianRed");

  /** 0x4B0082. */
  public static final CorrectedColor kIndigo = new CorrectedColor(0.29411766f, 0.0f, 0.50980395f, "kIndigo");

  /** 0xFFFFF0. */
  public static final CorrectedColor kIvory = new CorrectedColor(1.0f, 1.0f, 0.9411765f, "kIvory");

  /** 0xF0E68C. */
  public static final CorrectedColor kKhaki = new CorrectedColor(0.9411765f, 0.9019608f, 0.54901963f, "kKhaki");

  /** 0xE6E6FA. */
  public static final CorrectedColor kLavender = new CorrectedColor(0.9019608f, 0.9019608f, 0.98039216f, "kLavender");

  /** 0xFFF0F5. */
  public static final CorrectedColor kLavenderBlush =
      new CorrectedColor(1.0f, 0.9411765f, 0.9607843f, "kLavenderBlush");

  /** 0x7CFC00. */
  public static final CorrectedColor kLawnGreen = new CorrectedColor(0.4862745f, 0.9882353f, 0.0f, "kLawnGreen");

  /** 0xFFFACD. */
  public static final CorrectedColor kLemonChiffon =
      new CorrectedColor(1.0f, 0.98039216f, 0.8039216f, "kLemonChiffon");

  /** 0xADD8E6. */
  public static final CorrectedColor kLightBlue =
      new CorrectedColor(0.6784314f, 0.84705883f, 0.9019608f, "kLightBlue");

  /** 0xF08080. */
  public static final CorrectedColor kLightCoral =
      new CorrectedColor(0.9411765f, 0.5019608f, 0.5019608f, "kLightCoral");

  /** 0xE0FFFF. */
  public static final CorrectedColor kLightCyan = new CorrectedColor(0.8784314f, 1.0f, 1.0f, "kLightCyan");

  /** 0xFAFAD2. */
  public static final CorrectedColor kLightGoldenrodYellow =
      new CorrectedColor(0.98039216f, 0.98039216f, 0.8235294f, "kLightGoldenrodYellow");

  /** 0xD3D3D3. */
  public static final CorrectedColor kLightGray = new CorrectedColor(0.827451f, 0.827451f, 0.827451f, "kLightGray");

  /** 0x90EE90. */
  public static final CorrectedColor kLightGreen =
      new CorrectedColor(0.5647059f, 0.93333334f, 0.5647059f, "kLightGreen");

  /** 0xFFB6C1. */
  public static final CorrectedColor kLightPink = new CorrectedColor(1.0f, 0.7137255f, 0.75686276f, "kLightPink");

  /** 0xFFA07A. */
  public static final CorrectedColor kLightSalmon = new CorrectedColor(1.0f, 0.627451f, 0.47843137f, "kLightSalmon");

  /** 0x20B2AA. */
  public static final CorrectedColor kLightSeaGreen =
      new CorrectedColor(0.1254902f, 0.69803923f, 0.6666667f, "kLightSeaGreen");

  /** 0x87CEFA. */
  public static final CorrectedColor kLightSkyBlue =
      new CorrectedColor(0.5294118f, 0.80784315f, 0.98039216f, "kLightSkyBlue");

  /** 0x778899. */
  public static final CorrectedColor kLightSlateGray =
      new CorrectedColor(0.46666667f, 0.53333336f, 0.6f, "kLightSlateGray");

  /** 0xB0C4DE. */
  public static final CorrectedColor kLightSteelBlue =
      new CorrectedColor(0.6901961f, 0.76862746f, 0.87058824f, "kLightSteelBlue");

  /** 0xFFFFE0. */
  public static final CorrectedColor kLightYellow = new CorrectedColor(1.0f, 1.0f, 0.8784314f, "kLightYellow");

  /** 0x00FF00. */
  public static final CorrectedColor kLime = new CorrectedColor(0.0f, 1.0f, 0.0f, "kLime");

  /** 0x32CD32. */
  public static final CorrectedColor kLimeGreen =
      new CorrectedColor(0.19607843f, 0.8039216f, 0.19607843f, "kLimeGreen");

  /** 0xFAF0E6. */
  public static final CorrectedColor kLinen = new CorrectedColor(0.98039216f, 0.9411765f, 0.9019608f, "kLinen");

  /** 0xFF00FF. */
  public static final CorrectedColor kMagenta = new CorrectedColor(1.0f, 0.0f, 1.0f, "kMagenta");

  /** 0x800000. */
  public static final CorrectedColor kMaroon = new CorrectedColor(0.5019608f, 0.0f, 0.0f, "kMaroon");

  /** 0x66CDAA. */
  public static final CorrectedColor kMediumAquamarine =
      new CorrectedColor(0.4f, 0.8039216f, 0.6666667f, "kMediumAquamarine");

  /** 0x0000CD. */
  public static final CorrectedColor kMediumBlue = new CorrectedColor(0.0f, 0.0f, 0.8039216f, "kMediumBlue");

  /** 0xBA55D3. */
  public static final CorrectedColor kMediumOrchid =
      new CorrectedColor(0.7294118f, 0.33333334f, 0.827451f, "kMediumOrchid");

  /** 0x9370DB. */
  public static final CorrectedColor kMediumPurple =
      new CorrectedColor(0.5764706f, 0.4392157f, 0.85882354f, "kMediumPurple");

  /** 0x3CB371. */
  public static final CorrectedColor kMediumSeaGreen =
      new CorrectedColor(0.23529412f, 0.7019608f, 0.44313726f, "kMediumSeaGreen");

  /** 0x7B68EE. */
  public static final CorrectedColor kMediumSlateBlue =
      new CorrectedColor(0.48235294f, 0.40784314f, 0.93333334f, "kMediumSlateBlue");

  /** 0x00FA9A. */
  public static final CorrectedColor kMediumSpringGreen =
      new CorrectedColor(0.0f, 0.98039216f, 0.6039216f, "kMediumSpringGreen");

  /** 0x48D1CC. */
  public static final CorrectedColor kMediumTurquoise =
      new CorrectedColor(0.28235295f, 0.81960785f, 0.8f, "kMediumTurquoise");

  /** 0xC71585. */
  public static final CorrectedColor kMediumVioletRed =
      new CorrectedColor(0.78039217f, 0.08235294f, 0.52156866f, "kMediumVioletRed");

  /** 0x191970. */
  public static final CorrectedColor kMidnightBlue =
      new CorrectedColor(0.09803922f, 0.09803922f, 0.4392157f, "kMidnightBlue");

  /** 0xF5FFFA. */
  public static final CorrectedColor kMintcream = new CorrectedColor(0.9607843f, 1.0f, 0.98039216f, "kMintcream");

  /** 0xFFE4E1. */
  public static final CorrectedColor kMistyRose = new CorrectedColor(1.0f, 0.89411765f, 0.88235295f, "kMistyRose");

  /** 0xFFE4B5. */
  public static final CorrectedColor kMoccasin = new CorrectedColor(1.0f, 0.89411765f, 0.70980394f, "kMoccasin");

  /** 0xFFDEAD. */
  public static final CorrectedColor kNavajoWhite = new CorrectedColor(1.0f, 0.87058824f, 0.6784314f, "kNavajoWhite");

  /** 0x000080. */
  public static final CorrectedColor kNavy = new CorrectedColor(0.0f, 0.0f, 0.5019608f, "kNavy");

  /** 0xFDF5E6. */
  public static final CorrectedColor kOldLace = new CorrectedColor(0.99215686f, 0.9607843f, 0.9019608f, "kOldLace");

  /** 0x808000. */
  public static final CorrectedColor kOlive = new CorrectedColor(0.5019608f, 0.5019608f, 0.0f, "kOlive");

  /** 0x6B8E23. */
  public static final CorrectedColor kOliveDrab =
      new CorrectedColor(0.41960785f, 0.5568628f, 0.13725491f, "kOliveDrab");

  /** 0xFFA500. */
  public static final CorrectedColor kOrange = new CorrectedColor(1.0f, 0.64705884f, 0.0f, "kOrange");

  /** 0xFF4500. */
  public static final CorrectedColor kOrangeRed = new CorrectedColor(1.0f, 0.27058825f, 0.0f, "kOrangeRed");

  /** 0xDA70D6. */
  public static final CorrectedColor kOrchid = new CorrectedColor(0.85490197f, 0.4392157f, 0.8392157f, "kOrchid");

  /** 0xEEE8AA. */
  public static final CorrectedColor kPaleGoldenrod =
      new CorrectedColor(0.93333334f, 0.9098039f, 0.6666667f, "kPaleGoldenrod");

  /** 0x98FB98. */
  public static final CorrectedColor kPaleGreen =
      new CorrectedColor(0.59607846f, 0.9843137f, 0.59607846f, "kPaleGreen");

  /** 0xAFEEEE. */
  public static final CorrectedColor kPaleTurquoise =
      new CorrectedColor(0.6862745f, 0.93333334f, 0.93333334f, "kPaleTurquoise");

  /** 0xDB7093. */
  public static final CorrectedColor kPaleVioletRed =
      new CorrectedColor(0.85882354f, 0.4392157f, 0.5764706f, "kPaleVioletRed");

  /** 0xFFEFD5. */
  public static final CorrectedColor kPapayaWhip = new CorrectedColor(1.0f, 0.9372549f, 0.8352941f, "kPapayaWhip");

  /** 0xFFDAB9. */
  public static final CorrectedColor kPeachPuff = new CorrectedColor(1.0f, 0.85490197f, 0.7254902f, "kPeachPuff");

  /** 0xCD853F. */
  public static final CorrectedColor kPeru = new CorrectedColor(0.8039216f, 0.52156866f, 0.24705882f, "kPeru");

  /** 0xFFC0CB. */
  public static final CorrectedColor kPink = new CorrectedColor(1.0f, 0.7529412f, 0.79607844f, "kPink");

  /** 0xDDA0DD. */
  public static final CorrectedColor kPlum = new CorrectedColor(0.8666667f, 0.627451f, 0.8666667f, "kPlum");

  /** 0xB0E0E6. */
  public static final CorrectedColor kPowderBlue =
      new CorrectedColor(0.6901961f, 0.8784314f, 0.9019608f, "kPowderBlue");

  /** 0x800080. */
  public static final CorrectedColor kPurple = new CorrectedColor(0.5019608f, 0.0f, 0.5019608f, "kPurple");

  /** 0xFF0000. */
  public static final CorrectedColor kRed = new CorrectedColor(1.0f, 0.0f, 0.0f, "kRed");

  /** 0xBC8F8F. */
  public static final CorrectedColor kRosyBrown =
      new CorrectedColor(0.7372549f, 0.56078434f, 0.56078434f, "kRosyBrown");

  /** 0x4169E1. */
  public static final CorrectedColor kRoyalBlue =
      new CorrectedColor(0.25490198f, 0.4117647f, 0.88235295f, "kRoyalBlue");

  /** 0x8B4513. */
  public static final CorrectedColor kSaddleBrown =
      new CorrectedColor(0.54509807f, 0.27058825f, 0.07450981f, "kSaddleBrown");

  /** 0xFA8072. */
  public static final CorrectedColor kSalmon = new CorrectedColor(0.98039216f, 0.5019608f, 0.44705883f, "kSalmon");

  /** 0xF4A460. */
  public static final CorrectedColor kSandyBrown =
      new CorrectedColor(0.95686275f, 0.6431373f, 0.3764706f, "kSandyBrown");

  /** 0x2E8B57. */
  public static final CorrectedColor kSeaGreen =
      new CorrectedColor(0.18039216f, 0.54509807f, 0.34117648f, "kSeaGreen");

  /** 0xFFF5EE. */
  public static final CorrectedColor kSeashell = new CorrectedColor(1.0f, 0.9607843f, 0.93333334f, "kSeashell");

  /** 0xA0522D. */
  public static final CorrectedColor kSienna = new CorrectedColor(0.627451f, 0.32156864f, 0.1764706f, "kSienna");

  /** 0xC0C0C0. */
  public static final CorrectedColor kSilver = new CorrectedColor(0.7529412f, 0.7529412f, 0.7529412f, "kSilver");

  /** 0x87CEEB. */
  public static final CorrectedColor kSkyBlue = new CorrectedColor(0.5294118f, 0.80784315f, 0.92156863f, "kSkyBlue");

  /** 0x6A5ACD. */
  public static final CorrectedColor kSlateBlue =
      new CorrectedColor(0.41568628f, 0.3529412f, 0.8039216f, "kSlateBlue");

  /** 0x708090. */
  public static final CorrectedColor kSlateGray =
      new CorrectedColor(0.4392157f, 0.5019608f, 0.5647059f, "kSlateGray");

  /** 0xFFFAFA. */
  public static final CorrectedColor kSnow = new CorrectedColor(1.0f, 0.98039216f, 0.98039216f, "kSnow");

  /** 0x00FF7F. */
  public static final CorrectedColor kSpringGreen = new CorrectedColor(0.0f, 1.0f, 0.49803922f, "kSpringGreen");

  /** 0x4682B4. */
  public static final CorrectedColor kSteelBlue =
      new CorrectedColor(0.27450982f, 0.50980395f, 0.7058824f, "kSteelBlue");

  /** 0xD2B48C. */
  public static final CorrectedColor kTan = new CorrectedColor(0.8235294f, 0.7058824f, 0.54901963f, "kTan");

  /** 0x008080. */
  public static final CorrectedColor kTeal = new CorrectedColor(0.0f, 0.5019608f, 0.5019608f, "kTeal");

  /** 0xD8BFD8. */
  public static final CorrectedColor kThistle = new CorrectedColor(0.84705883f, 0.7490196f, 0.84705883f, "kThistle");

  /** 0xFF6347. */
  public static final CorrectedColor kTomato = new CorrectedColor(1.0f, 0.3882353f, 0.2784314f, "kTomato");

  /** 0x40E0D0. */
  public static final CorrectedColor kTurquoise =
      new CorrectedColor(0.2509804f, 0.8784314f, 0.8156863f, "kTurquoise");

  /** 0xEE82EE. */
  public static final CorrectedColor kViolet = new CorrectedColor(0.93333334f, 0.50980395f, 0.93333334f, "kViolet");

  /** 0xF5DEB3. */
  public static final CorrectedColor kWheat = new CorrectedColor(0.9607843f, 0.87058824f, 0.7019608f, "kWheat");

  /** 0xFFFFFF. */
  public static final CorrectedColor kWhite = new CorrectedColor(1.0f, 1.0f, 1.0f, "kWhite");

  /** 0xF5F5F5. */
  public static final CorrectedColor kWhiteSmoke =
      new CorrectedColor(0.9607843f, 0.9607843f, 0.9607843f, "kWhiteSmoke");

  /** 0xFFFF00. */
  public static final CorrectedColor kYellow = new CorrectedColor(1.0f, 1.0f, 0.0f, "kYellow");

  /** 0x9ACD32. */
  public static final CorrectedColor kYellowGreen =
      new CorrectedColor(0.6039216f, 0.8039216f, 0.19607843f, "kYellowGreen");
}
