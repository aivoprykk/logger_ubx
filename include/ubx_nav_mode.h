#ifndef B9FA99ED_851F_4057_A758_BA82A663C3F6
#define B9FA99ED_851F_4057_A758_BA82A663C3F6
#ifndef UBX_NAV_MODE_H
#define UBX_NAV_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "config_ubx.h"

ubx_nav_mode_t ubx_nav_mode_get_base(void);
ubx_nav_mode_t ubx_nav_mode_get_effective(void);
bool ubx_nav_mode_is_override_active(void);
bool ubx_nav_mode_update_from_speed(float speed_2s, ubx_nav_mode_t *old_mode,
					ubx_nav_mode_t *new_mode);
void ubx_nav_mode_on_base_mode_changed(void);
void ubx_nav_mode_on_session_end(void);

#ifdef __cplusplus
}
#endif

#endif


#endif /* B9FA99ED_851F_4057_A758_BA82A663C3F6 */
