#pragma once
/*--------------------------------------------------------------------------------------
 DMD_SPWM_RegProfiles.h - register profile catalog support (Register Config Finder)

 Profile fragments live in src/register_profiles/<chip>/<chip>_scan_<N>.h and are
 included *inside* an array of DMD_SPWM_RegProfile<MAX_WORDS>:

   #include <DMD_SPWM_RegProfiles.h>
   #define DMD_SPWM_REGISTER_TEST_PROFILE_CHIP DMD_SPWM_REGISTER_TEST_PROFILE_CHIP_FM6373
   static const DMD_SPWM_FM6373_Profile profiles[] = {
   #include "register_profiles/fm6373/fm6373_scan_32.h"
   };

 Then: dmdApplyRegProfile(dmd, profiles[i], true);
--------------------------------------------------------------------------------------*/
#include "DMD_SPWM.h"

#define DMD_SPWM_REGISTER_TEST_BUILDING_PROFILE_ARRAY 1
#define DMD_SPWM_REGISTER_TEST_PROFILE_CHIP_FM6353    1
#define DMD_SPWM_REGISTER_TEST_PROFILE_CHIP_FM6363    2
#define DMD_SPWM_REGISTER_TEST_PROFILE_CHIP_FM6373    3
#define DMD_SPWM_REGISTER_TEST_PROFILE_CHIP_ICND1065L 4
#define DMD_SPWM_REGISTER_TEST_PROFILE_CHIP_SM16380SH 5
#define DMD_SPWM_REGISTER_TEST_PROFILE_CHIP_DP3264    6
#define DMD_SPWM_REGISTER_TEST_PROFILE_CHIP_ICND2055  7

// catalog entry: displayed REG number, word count, Red / Green / Blue lane words
template <uint8_t MaxWords>
struct DMD_SPWM_RegProfile {
	enum { MAX_WORDS = MaxWords };
	uint16_t catalog_index;
	uint8_t word_count;
	uint16_t channel_words[3][MaxWords];
};

typedef DMD_SPWM_RegProfile<5>  DMD_SPWM_FM6353_Profile;
typedef DMD_SPWM_RegProfile<5>  DMD_SPWM_FM6363_Profile;
typedef DMD_SPWM_RegProfile<47> DMD_SPWM_FM6373_Profile;
typedef DMD_SPWM_RegProfile<22> DMD_SPWM_ICND1065L_Profile;
typedef DMD_SPWM_RegProfile<47> DMD_SPWM_ICND2055_Profile;
typedef DMD_SPWM_RegProfile<32> DMD_SPWM_SM16380SH_Profile;
typedef DMD_SPWM_RegProfile<13> DMD_SPWM_DP3264_Profile;

// Load a profile into the driver and send it to the panel.
// rgb_lanes = true : separate R/G/B words; false : Red words to all lanes.
template <uint8_t N>
bool dmdApplyRegProfile(DMD_SPWM& dmd, const DMD_SPWM_RegProfile<N>& p, bool rgb_lanes = true,
	uint16_t delay_ms = 30)
{
	if (p.word_count == 0 || p.word_count > N) return false;
	if (!dmd.setConfigRegisters(p.channel_words[0], p.word_count,
		rgb_lanes ? p.channel_words[1] : nullptr,
		rgb_lanes ? p.channel_words[2] : nullptr)) return false;
	dmd.applyConfigRegisters(delay_ms);
	return true;
}

// Find a profile by its REG number; returns nullptr if not compiled in.
template <uint8_t N, size_t COUNT>
const DMD_SPWM_RegProfile<N>* dmdFindRegProfile(const DMD_SPWM_RegProfile<N>(&list)[COUNT], uint16_t reg)
{
	for (size_t i = 0; i < COUNT; i++) if (list[i].catalog_index == reg) return &list[i];
	return nullptr;
}
