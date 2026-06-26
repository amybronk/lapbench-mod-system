#define DEBUG_SERIAL       // Commentaar = geen debug output
#ifdef DEBUG_SERIAL
  #define DBG(x)   Serial.println(x)
  #define DBGF(...)Serial.printf(__VA_ARGS__)
#else
  #define DBG(x)
  #define DBGF(...)
#endif


//DBGF("[SCN] Ronde %d klaar\n", var);
// %d word veranderd met var