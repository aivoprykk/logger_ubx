#include "ubx_nav_mode.h"

#if defined(CONFIG_UBLOX_ENABLED)

#include "logger_common.h"
#include "unified_config.h"

enum {
	NAV_MODE_PED_TO_PORT_SPEED_MM_S = 29972,
	NAV_MODE_PORT_TO_PED_SPEED_MM_S = 29694,
	NAV_MODE_SEA_TO_PORT_SPEED_MM_S = 23750,
	NAV_MODE_PORT_TO_SEA_SPEED_MM_S = 21750,
};

typedef struct ubx_nav_mode_state_s {
	ubx_nav_mode_t base_mode;
	bool override_active;
} ubx_nav_mode_state_t;

static ubx_nav_mode_state_t s_nav_mode_state = {
	.base_mode = UBX_MODE_PEDESTRIAN,
	.override_active = false,
};

static ubx_nav_mode_t get_nav_mode_for_state(ubx_nav_mode_t nav_mode,
						     bool override_active) {
	if (!override_active)
		return nav_mode;

	if (nav_mode == UBX_MODE_PEDESTRIAN || nav_mode == UBX_MODE_SEA)
		return UBX_MODE_PORTABLE;

	if (nav_mode == UBX_MODE_PORTABLE)
		return UBX_MODE_PEDESTRIAN;

	return nav_mode;
}

static bool nav_mode_override_should_be_active(ubx_nav_mode_t nav_mode,
						       bool override_active,
						       float speed_2s) {
	switch (nav_mode) {
	case UBX_MODE_PEDESTRIAN:
		return override_active ? speed_2s >= NAV_MODE_PORT_TO_PED_SPEED_MM_S
					       : speed_2s > NAV_MODE_PED_TO_PORT_SPEED_MM_S;
	case UBX_MODE_SEA:
		return override_active ? speed_2s >= NAV_MODE_PORT_TO_SEA_SPEED_MM_S
					       : speed_2s > NAV_MODE_SEA_TO_PORT_SPEED_MM_S;
	case UBX_MODE_PORTABLE:
		return override_active ? speed_2s <= NAV_MODE_PED_TO_PORT_SPEED_MM_S
					       : speed_2s < NAV_MODE_PORT_TO_PED_SPEED_MM_S;
	default:
		return false;
	}
}

static void ubx_nav_mode_sync_base(void) {
	const ubx_nav_mode_t base_mode = g_rtc_config.ubx.nav_mode;

	if (s_nav_mode_state.base_mode != base_mode) {
		s_nav_mode_state.base_mode = base_mode;
		s_nav_mode_state.override_active = false;
	}
}

ubx_nav_mode_t ubx_nav_mode_get_base(void) {
	return g_rtc_config.ubx.nav_mode;
}

ubx_nav_mode_t ubx_nav_mode_get_effective(void) {
	ubx_nav_mode_sync_base();
	return get_nav_mode_for_state(s_nav_mode_state.base_mode,
				      s_nav_mode_state.override_active);
}

bool ubx_nav_mode_is_override_active(void) {
	ubx_nav_mode_sync_base();
	return s_nav_mode_state.override_active;
}

bool ubx_nav_mode_update_from_speed(float speed_2s, ubx_nav_mode_t *old_mode,
					ubx_nav_mode_t *new_mode) {
	ubx_nav_mode_sync_base();

	const ubx_nav_mode_t base_mode = s_nav_mode_state.base_mode;
	const ubx_nav_mode_t current_mode = get_nav_mode_for_state(
		base_mode, s_nav_mode_state.override_active);
	const bool next_override_active = nav_mode_override_should_be_active(
		base_mode, s_nav_mode_state.override_active, speed_2s);
	const ubx_nav_mode_t updated_mode = get_nav_mode_for_state(
		base_mode, next_override_active);

	s_nav_mode_state.override_active = next_override_active;

	if (old_mode)
		*old_mode = current_mode;
	if (new_mode)
		*new_mode = updated_mode;

	return current_mode != updated_mode;
}

void ubx_nav_mode_on_base_mode_changed(void) {
	s_nav_mode_state.base_mode = g_rtc_config.ubx.nav_mode;
	s_nav_mode_state.override_active = false;
}

void ubx_nav_mode_on_session_end(void) {
	ubx_nav_mode_on_base_mode_changed();
}

#endif