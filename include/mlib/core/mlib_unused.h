/// mute unused variable warning
#define MLIB_UNUSED(x) \
  do{                  \
      (void)(x);         \
    } while (0);
