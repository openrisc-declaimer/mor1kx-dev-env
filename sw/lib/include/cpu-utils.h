#ifndef _CPU_UTILS_H_
#define _CPU_UTILS_H_

#ifdef OPENRISC_CPU_TYPE_MOK1KX
# include "mor1kx-utils.h"
#else
# ifdef OPENRISC_CPU_TYPE_OR1200
#  include "or1200-utils.h"
# endif /* OPENRISC_CPU_TYPE_OR1200 */
#endif /* OPENRISC_CPU_TYPE_MOK1KX */

#endif
