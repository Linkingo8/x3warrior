#ifndef __WARRIOR_CONTROLLER__WARRIOR_CONTROLLER_COMPILER_H__
#define __WARRIOR_CONTROLLER__WARRIOR_CONTROLLER_COMPILER_H__

#if defined _WIN32 || defined __CYGWIN__
#   ifdef __GNUC__
#       define WARRIOR_CONTROLLER_EXPORT __attribute__((dllexport))
#       define WARRIOR_CONTROLLER_IMPORT __attribute__((dllimport))
#   else
#       define WARRIOR_CONTROLLER_EXPORT __declspec(dllexport)
#       define WARRIOR_CONTROLLER_IMPORT __declspec(dllimport)
#   endif
#   ifdef WARRIOR_CONTROLLER_BUILDING_DLL
#       define WARRIOR_CONTROLLER_PUBLIC WARRIOR_CONTROLLER_EXPORT
#   else
#       define WARRIOR_CONTROLLER_PUBLIC WARRIOR_CONTROLLER_IMPORT
#   endif
#   define WARRIOR_CONTROLLER_PUBLIC_TYPE WARRIOR_CONTROLLER_PUBLIC
#   define WARRIOR_CONTROLLER_LOCAL
#else
#   define WARRIOR_CONTROLLER_EXPORT __attribute__((visibility("default")))
#   define WARRIOR_CONTROLLER_IMPORT
#   if __GNUC__ >= 4
#       define WARRIOR_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#       define WARRIOR_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#   else
#       define WARRIOR_CONTROLLER_PUBLIC
#       define WARRIOR_CONTROLLER_LOCAL
#   endif
#   define WARRIOR_CONTROLLER_PUBLIC_TYPE
#endif

#endif // __WARRIOR_CONTROLLER__WARRIOR_CONTROLLER_COMPILER_H__
