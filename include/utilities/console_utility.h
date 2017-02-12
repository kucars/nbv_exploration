#ifndef CONSOLE_UTILITY_H
#define CONSOLE_UTILITY_H


struct ConsoleUtility {
  ConsoleUtility():
    black  ("\033[0;30m"),
    red    ("\033[0;31m"),
    green  ("\033[1;32m"),
    yellow ("\033[1;33m"),
    blue   ("\033[1;34m"),
    magenta("\033[0;35m"),
    cyan   ("\033[0;36m"),
    white  ("\033[0;37m"),

    bold      ("\033[1m"),
    darken    ("\033[2m"),
    underline ("\033[4m"),
    background("\033[7m"),
    strike    ("\033[9m"),

    erase_line("\033[2K"),
    reset     ("\033[0m")
    { }   // default Constructor

  const std::string black, red, green, yellow, blue, magenta, cyan, white;
  const std::string bold, darken, underline, background, strike;
  const std::string erase_line, reset;
};


#endif // CONSOLE_UTILITY_H
