/* Build shim - NOT the upstream Configure.h.
 *
 * Commands.h only consumes Configure.h for the CPU-type macros, which select
 * MAX_COMMAND_SIZE. The values are arbitrary but must be distinct; CPU=JMxx
 * matches the JMxx/JS16-class firmware the FZ0622C runs (254-byte commands).
 */
#ifndef _CONFIGURE_H_
#define _CONFIGURE_H_

#define JB16    1
#define JMxx    2
#define UF32    3
#define MKL25Z4 4
#define MK20D5  5

#define CPU JMxx

#endif
