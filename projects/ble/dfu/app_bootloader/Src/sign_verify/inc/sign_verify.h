#ifndef _SIGN_VERIFY_H_
#define _SIGN_VERIFY_H_


bool sign_verify(uint32_t fw_start_addr, uint32_t fw_size, const uint8_t *p_public_key_hash, uint32_t is_sec_enable);



#endif


