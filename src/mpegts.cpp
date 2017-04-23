#include "mpegts.hpp"
#include <iostream>
#include <cstring>
#include <algorithm>
#include <boost/assert.hpp>

namespace util {

#ifndef AV_RB32
#   define AV_RB32(x)                                \
	(((uint32_t)((const uint8_t*)(x))[0] << 24) |    \
			   (((const uint8_t*)(x))[1] << 16) |    \
			   (((const uint8_t*)(x))[2] <<  8) |    \
				((const uint8_t*)(x))[3])
#endif

	enum AVRounding {
		AV_ROUND_ZERO = 0, ///< Round toward zero.
		AV_ROUND_INF = 1, ///< Round away from zero.
		AV_ROUND_DOWN = 2, ///< Round toward -infinity.
		AV_ROUND_UP = 3, ///< Round toward +infinity.
		AV_ROUND_NEAR_INF = 5, ///< Round to nearest and halfway cases away from zero.
		AV_ROUND_PASS_MINMAX = 8192, ///< Flag to pass INT64_MIN/MAX through instead of rescaling, this avoids special cases for AV_NOPTS_VALUE
	};

	int64_t av_rescale_rnd(int64_t a, int64_t b, int64_t c, int rnd)
	{
		int64_t r = 0;
		BOOST_ASSERT(c > 0);
		BOOST_ASSERT(b >= 0);
		BOOST_ASSERT((unsigned)(rnd&~AV_ROUND_PASS_MINMAX) <= 5 && (rnd&~AV_ROUND_PASS_MINMAX) != 4);

		if (c <= 0 || b < 0 || !((unsigned)(rnd&~AV_ROUND_PASS_MINMAX) <= 5 && (rnd&~AV_ROUND_PASS_MINMAX) != 4))
			return INT64_MIN;

		if (rnd & AV_ROUND_PASS_MINMAX) {
			if (a == INT64_MIN || a == INT64_MAX)
				return a;
			rnd -= AV_ROUND_PASS_MINMAX;
		}

		if (a < 0)
			return -(uint64_t)av_rescale_rnd(-std::max(a, -INT64_MAX), b, c, rnd ^ ((rnd >> 1) & 1));

		if (rnd == AV_ROUND_NEAR_INF)
			r = c / 2;
		else if (rnd & 1)
			r = c - 1;

		if (b <= INT_MAX && c <= INT_MAX) {
			if (a <= INT_MAX)
				return (a * b + r) / c;
			else {
				int64_t ad = a / c;
				int64_t a2 = (a % c * b + r) / c;
				if (ad >= INT32_MAX && b && ad > (INT64_MAX - a2) / b)
					return INT64_MIN;
				return ad * b + a2;
			}
		}
		else {
#if 1
			uint64_t a0 = a & 0xFFFFFFFF;
			uint64_t a1 = a >> 32;
			uint64_t b0 = b & 0xFFFFFFFF;
			uint64_t b1 = b >> 32;
			uint64_t t1 = a0 * b1 + a1 * b0;
			uint64_t t1a = t1 << 32;
			int i;

			a0 = a0 * b0 + t1a;
			a1 = a1 * b1 + (t1 >> 32) + (a0 < t1a);
			a0 += r;
			a1 += a0 < r;

			for (i = 63; i >= 0; i--) {
				a1 += a1 + ((a0 >> i) & 1);
				t1 += t1;
				if (c <= a1) {
					a1 -= c;
					t1++;
				}
			}
			if (t1 > INT64_MAX)
				return INT64_MIN;
			return t1;
		}
#else
			AVInteger ai;
			ai = av_mul_i(av_int2i(a), av_int2i(b));
			ai = av_add_i(ai, av_int2i(r));

			return av_i2int(av_div_i(ai, av_int2i(c)));
	}
#endif
}

	int64_t av_rescale(int64_t a, int64_t b, int64_t c)
	{
		return av_rescale_rnd(a, b, c, AV_ROUND_NEAR_INF);
	}

#define PCR_TIME_BASE				27000000
#define TS_SIZE						188
#define TS_HEADER_SIZE				4
#define TS_HEADER_SIZE_AF			6
#define TS_HEADER_SIZE_PCR			12
#define PSI_HEADER_SIZE				3
#define PSI_HEADER_SIZE_SYNTAX1		8
#define PSI_CRC_SIZE				4
#define PAT_HEADER_SIZE				PSI_HEADER_SIZE_SYNTAX1
#define PAT_PROGRAM_SIZE			4
#define PMT_HEADER_SIZE			    (PSI_HEADER_SIZE_SYNTAX1 + 4)
#define PMT_ES_SIZE					5
#define PES_HEADER_SIZE				6
#define PES_HEADER_OPTIONAL_SIZE	3

	inline bool ts_get_unitstart(const uint8_t* ts)
	{
		return !!(ts[1] & 0x40);
	}

	inline bool ts_has_payload(const uint8_t* ts)
	{
		return !!(ts[3] & 0x10);
	}

	inline bool ts_has_adaptation(const uint8_t* ts)
	{
		return !!(ts[3] & 0x20);
	}

	inline uint8_t ts_get_adaptation_length(const uint8_t* ts)
	{
		return ts[4];
	}

	inline uint16_t psi_get_length(const uint8_t* section)
	{
		return ((section[1] & 0xf) << 8) | section[2];
	}

	inline uint16_t pmt_get_desclength(const uint8_t *p)
	{
		return ((p[10] & 0xf) << 8) | p[11];
	}

	inline uint16_t pmtn_get_desclength(const uint8_t *p)
	{
		return ((p[3] & 0xf) << 8) | p[4];
	}

	inline uint8_t* pmt_get_es(uint8_t *p, uint8_t n)
	{
		uint16_t section_size = psi_get_length(p) + PSI_HEADER_SIZE
			- PSI_CRC_SIZE;
		uint8_t *pn = p + PMT_HEADER_SIZE + pmt_get_desclength(p);
		if (pn - p > section_size) return NULL;

		while (n) {
			if (pn + PMT_ES_SIZE - p > section_size) return NULL;
			pn += PMT_ES_SIZE + pmtn_get_desclength(pn);
			n--;
		}
		if (pn - p >= section_size) return NULL;
		return pn;
	}

	inline void ts_set_pid(uint8_t* ts, uint16_t pid)
	{
		ts[0] = 0x47;
		ts[1] = 0x0;
		ts[2] = 0x0;
		ts[3] = 0x0;
		ts[4] = 0x0;	// pointer_field.
		ts[1] &= ~0x1f;
		ts[1] |= (pid >> 8) & 0x1f;
		ts[2] = pid & 0xff;
	}

	inline void ts_set_transportpriority(uint8_t* ts)
	{
		ts[1] |= 0x20;
	}

	inline void ts_set_payload(uint8_t* ts)
	{
		ts[3] |= 0x10;
	}

	inline void ts_set_unitstart(uint8_t* ts)
	{
		ts[1] |= 0x40;
	}

	inline void ts_set_cc(uint8_t* ts, uint8_t cc)
	{
		ts[3] &= ~0xf;
		ts[3] |= (cc & 0xf);
	}

	inline void ts_set_adaptation(uint8_t* ts, uint8_t length)
	{
		ts[3] |= 0x20;
		ts[4] = length;
		if (length)
			ts[5] = 0x0;
		if (length > 1)
			memset(&ts[6], 0xff, length - 1); /* stuffing */
	}

	inline void ts_set_scrambling(uint8_t* ts, uint8_t scrambling)
	{
		ts[3] &= ~0xc0;
		ts[3] |= scrambling << 6;
	}

	inline uint8_t* ts_payload(uint8_t* ts)
	{
		if (!ts_has_payload(ts))
			return ts + TS_SIZE;
		if (!ts_has_adaptation(ts))
			return ts + TS_HEADER_SIZE;
		return ts + TS_HEADER_SIZE + 1 + ts_get_adaptation_length(ts);
	}

	inline uint8_t* ts_section(uint8_t* ts)
	{
		if (!ts_get_unitstart(ts))
			return ts_payload(ts);

		return ts_payload(ts) + 1; /* pointer_field */
	}

	inline uint8_t* ts_adaptation_field(uint8_t* ts)
	{
		if (ts_has_adaptation(ts))
			return ts + 5;
		return nullptr;
	}

	inline void psi_set_tableid(uint8_t* section, uint8_t table_id)
	{
		section[0] = table_id;
	}

	inline void psi_set_syntax(uint8_t* section)
	{
		section[1] = 0x70;
		section[1] |= 0x80;
		section[5] = 0xc0;
	}

	inline void psi_set_length(uint8_t* section, uint16_t length)
	{
		section[1] &= ~0xf;
		section[1] |= (length >> 8) & 0xf;
		section[2] = length & 0xff;
	}

	inline void psi_set_number(uint8_t* section, uint16_t n)
	{
		section[3] = n >> 8;
		section[4] = n & 0xff;
	}

	inline void psi_set_current(uint8_t* section)
	{
		section[5] |= 0x1;
	}

	inline void psi_set_version(uint8_t* section, uint8_t version)
	{
		section[5] = (version << 1) | 0xc0;
	}

	inline void psi_set_section(uint8_t* section, uint8_t n)
	{
		section[6] = n;
	}

	inline void psi_set_lastsection(uint8_t* section, uint8_t last_section)
	{
		section[7] = last_section;
	}

	inline void psi_set_crc(uint8_t* section)
	{
		uint32_t crc = 0xffffffff;
		uint16_t end = (((section[1] & 0xf) << 8) | section[2])
			+ PSI_HEADER_SIZE - PSI_CRC_SIZE;

		crc = crc32(section, end);

		section[end] = crc >> 24;
		section[end + 1] = (crc >> 16) & 0xff;
		section[end + 2] = (crc >> 8) & 0xff;
		section[end + 3] = crc & 0xff;
	}

	inline void psi_set_end(uint8_t* section)
	{
		uint16_t end = (((section[1] & 0xf) << 8) | section[2])
			+ PSI_HEADER_SIZE;
		auto length = TS_SIZE - (TS_HEADER_SIZE + end + PSI_CRC_SIZE);
		memset(section + end, 0xff, length);
	}

	inline uint8_t* pat_get_program(uint8_t* pat, uint8_t n)
	{
		uint8_t *pat_n = pat + PAT_HEADER_SIZE + n * PAT_PROGRAM_SIZE;
		if (pat_n + PAT_PROGRAM_SIZE - pat >
			psi_get_length(pat) + PSI_HEADER_SIZE - PSI_CRC_SIZE)
			return nullptr;
		return pat_n;
	}

	inline void patn_set_program(uint8_t* pat_n, uint16_t program)
	{
		pat_n[0] = program >> 8;
		pat_n[1] = program & 0xff;
	}

	inline void patn_set_pid(uint8_t* pat_n, uint16_t pid)
	{
		pat_n[2] &= ~0x1f;
		pat_n[2] |= pid >> 8;
		pat_n[3] = pid & 0xff;
	}

	inline void pmt_set_pcrpid(uint8_t* p, uint16_t pcr_pid)
	{
		p[8] &= ~0x1f;
		p[8] |= pcr_pid >> 8;
		p[9] = pcr_pid & 0xff;
	}

	inline void pmt_set_desclength(uint8_t* p, uint16_t length)
	{
		p[10] &= ~0xf;
		p[10] |= length >> 8;
		p[11] = length & 0xff;
	}

	inline void pmtn_init(uint8_t *pmtn)
	{
		pmtn[1] = 0xe0;
		pmtn[3] = 0xf0;
	}

	inline void pmtn_set_streamtype(uint8_t* pmtn, uint8_t stream_type)
	{
		pmtn[0] = stream_type;
	}

	inline void pmtn_set_pid(uint8_t* pmtn, uint16_t pid)
	{
		pmtn[1] &= ~0x1f;
		pmtn[1] |= pid >> 8;
		pmtn[2] = pid & 0xff;
	}

	inline void tsaf_set_discontinuity(uint8_t* ts)
	{
		ts[5] |= 0x80;
	}

	inline void tsaf_set_randomaccess(uint8_t* ts)
	{
		ts[5] |= 0x40;
	}

	inline void tsaf_set_streampriority(uint8_t* ts)
	{
		ts[5] |= 0x20;
	}

	inline void tsaf_set_pcr(uint8_t* ts, uint64_t pcr)
	{
		ts[5] |= 0x10;
		ts[6] = (pcr >> 25) & 0xff;
		ts[7] = (pcr >> 17) & 0xff;
		ts[8] = (pcr >> 9) & 0xff;
		ts[9] = (pcr >> 1) & 0xff;
		ts[10] = 0x7e | ((pcr << 7) & 0x80);
		ts[11] = 0;
	}

	inline void tsaf_set_pcrext(uint8_t* ts, uint16_t pcr_ext)
	{
		ts[10] |= (pcr_ext >> 8) & 0x1;
		ts[11] = pcr_ext & 0xff;
	}

	inline void pes_init(uint8_t* pes)
	{
		pes[0] = 0x0;
		pes[1] = 0x0;
		pes[2] = 0x1;
	}

	inline void pes_set_streamid(uint8_t* pes, uint8_t stream_id)
	{
		pes[3] = stream_id;
	}

	inline void pes_set_length(uint8_t* pes, uint16_t length)
	{
		pes[4] = length >> 8;
		pes[5] = length & 0xff;
	}

	inline void pes_set_headerlength(uint8_t* pes, uint8_t length)
	{
		pes[6] = 0x80;
		pes[7] = 0x0;
		pes[8] = length;
		if (length > 0)
			memset(&pes[9], 0xff, length); /* stuffing */
	}

	inline void pes_set_dataalignment(uint8_t* pes)
	{
		pes[6] |= 0x4;
	}

	inline bool pes_has_pts(const uint8_t* pes)
	{
		return !!(pes[7] & 0x80);
	}

	inline bool pes_has_dts(const uint8_t* pes)
	{
		return (pes[7] & 0xc0) == 0xc0;
	}

	inline void pes_set_pts(uint8_t* pes, uint64_t pts)
	{
		pes[7] |= 0x80;
		if (pes[8] < 5)
			pes[8] = 5;
		uint8_t marker = pes_has_dts(pes) ? 0x30 : 0x20;
		pes[9] = marker | 0x1 | ((pts >> 29) & 0xe);
		pes[10] = (pts >> 22) & 0xff;
		pes[11] = 0x1 | ((pts >> 14) & 0xfe);
		pes[12] = (pts >> 7) & 0xff;
		pes[13] = 0x1 | ((pts << 1) & 0xfe);
	}

	inline void pes_set_dts(uint8_t* pes, uint64_t dts)
	{
		pes[7] |= 0x40;
		if (pes[8] < 10)
			pes[8] = 10;
		pes[9] &= 0x0f;
		pes[9] |= 0x30;
		pes[14] = 0x11 | ((dts >> 29) & 0xe);
		pes[15] = (dts >> 22) & 0xff;
		pes[16] = 0x1 | ((dts >> 14) & 0xfe);
		pes[17] = (dts >> 7) & 0xff;
		pes[18] = 0x1 | ((dts << 1) & 0xfe);
	}

	inline uint8_t pes_get_headerlength(const uint8_t* pes)
	{
		return pes[8];
	}

	inline uint8_t* pes_payload(uint8_t* pes)
	{
		return pes + PES_HEADER_SIZE + PES_HEADER_OPTIONAL_SIZE + pes_get_headerlength(pes);
	}

	static const uint32_t static_crc_table[256] = {
		0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9,
		0x130476dc, 0x17c56b6b, 0x1a864db2, 0x1e475005,
		0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61,
		0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd,
		0x4c11db70, 0x48d0c6c7, 0x4593e01e, 0x4152fda9,
		0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
		0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011,
		0x791d4014, 0x7ddc5da3, 0x709f7b7a, 0x745e66cd,
		0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039,
		0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5,
		0xbe2b5b58, 0xbaea46ef, 0xb7a96036, 0xb3687d81,
		0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
		0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49,
		0xc7361b4c, 0xc3f706fb, 0xceb42022, 0xca753d95,
		0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1,
		0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d,
		0x34867077, 0x30476dc0, 0x3d044b19, 0x39c556ae,
		0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
		0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16,
		0x018aeb13, 0x054bf6a4, 0x0808d07d, 0x0cc9cdca,
		0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde,
		0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02,
		0x5e9f46bf, 0x5a5e5b08, 0x571d7dd1, 0x53dc6066,
		0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
		0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e,
		0xbfa1b04b, 0xbb60adfc, 0xb6238b25, 0xb2e29692,
		0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6,
		0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a,
		0xe0b41de7, 0xe4750050, 0xe9362689, 0xedf73b3e,
		0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
		0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686,
		0xd5b88683, 0xd1799b34, 0xdc3abded, 0xd8fba05a,
		0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637,
		0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb,
		0x4f040d56, 0x4bc510e1, 0x46863638, 0x42472b8f,
		0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
		0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47,
		0x36194d42, 0x32d850f5, 0x3f9b762c, 0x3b5a6b9b,
		0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff,
		0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623,
		0xf12f560e, 0xf5ee4bb9, 0xf8ad6d60, 0xfc6c70d7,
		0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
		0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f,
		0xc423cd6a, 0xc0e2d0dd, 0xcda1f604, 0xc960ebb3,
		0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7,
		0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b,
		0x9b3660c6, 0x9ff77d71, 0x92b45ba8, 0x9675461f,
		0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
		0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640,
		0x4e8ee645, 0x4a4ffbf2, 0x470cdd2b, 0x43cdc09c,
		0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8,
		0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24,
		0x119b4be9, 0x155a565e, 0x18197087, 0x1cd86d30,
		0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
		0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088,
		0x2497d08d, 0x2056cd3a, 0x2d15ebe3, 0x29d4f654,
		0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0,
		0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c,
		0xe3a1cbc1, 0xe760d676, 0xea23f0af, 0xeee2ed18,
		0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
		0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0,
		0x9abc8bd5, 0x9e7d9662, 0x933eb0bb, 0x97ffad0c,
		0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668,
		0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4
	};

	uint32_t crc32(const uint8_t* data, size_t len)
	{
		uint32_t crc = 0xffffffff;
		for (int i = 0; i < len; i++)
			crc = (crc << 8) ^ static_crc_table[(crc >> 24) ^ (data[i])];
		return crc;
	}

	const uint8_t ts_log2_tab[256] = {
			0,0,1,1,2,2,2,2,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
			5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
			6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
			6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
			7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
			7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
			7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
			7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7
	};

	const uint8_t ts_h264_golomb_to_pict_type[5] = {
		av_picture_type_p, av_picture_type_b, av_picture_type_i,
		av_picture_type_sp, av_picture_type_si
	};

	static inline const int ts_log2(unsigned int v)
	{
		int n = 0;
		if (v & 0xffff0000) {
			v >>= 16;
			n += 16;
		}
		if (v & 0xff00) {
			v >>= 8;
			n += 8;
		}
		n += ts_log2_tab[v];

		return n;
	}

	class bitstream
	{
		// c++11 noncopyable.
		bitstream(const bitstream&) = delete;
		bitstream& operator=(const bitstream&) = delete;

	public:
		bitstream(const uint8_t * data, int size)
			: m_data(data)
			, m_end(data + size)
			, m_head(0)
			, m_cache(0xffffffff)
		{}
		~bitstream()
		{}

	public:
		inline uint32_t read(int n)
		{
			uint32_t res = 0;
			int shift;

			if (n == 0)
				return res;

			/* fill up the cache if we need to */
			while (m_head < n) {
				uint8_t byte;
				bool check_three_byte;
				check_three_byte = true;
			next_byte:
				if (m_data >= m_end) {
					/* we're at the end, can't produce more than head number of bits */
					n = m_head;
					break;
				}
				/* get the byte, this can be an emulation_prevention_three_byte that we need
				* to ignore. */
				byte = *m_data++;
				if (check_three_byte && byte == 0x03 && ((m_cache & 0xffff) == 0)) {
					/* next byte goes unconditionally to the cache, even if it's 0x03 */
					check_three_byte = false;
					goto next_byte;
				}
				/* shift bytes in cache, moving the head bits of the cache left */
				m_cache = (m_cache << 8) | byte;
				m_head += 8;
			}

			/* bring the required bits down and truncate */
			if ((shift = m_head - n) > 0)
				res = static_cast<uint32_t>(m_cache >> shift);
			else
				res = static_cast<uint32_t>(m_cache);

			/* mask out required bits */
			if (n < 32)
				res &= (1 << n) - 1;

			m_head = shift;

			return res;
		}

		inline bool eos()
		{
			return (m_data >= m_end) && (m_head == 0);
		}

		inline int read_ue()
		{
			int i = 0;

			while (read(1) == 0 && !eos() && i < 32)
				i++;

			return ((1 << i) - 1 + read(i));
		}

		inline int read_se()
		{
			int i = 0;

			i = read_ue();
			/* (-1)^(i+1) Ceil (i / 2) */
			i = (i + 1) / 2 * (i & 1 ? 1 : -1);

			return i;
		}

	private:
		const uint8_t* m_data;
		const uint8_t* m_end;
		int m_head;
		uint64_t m_cache;
	};

	static inline const uint8_t *find_start_code(const uint8_t * p,
		const uint8_t *end, uint32_t * state)
	{
		int i;

		BOOST_ASSERT(p <= end);
		if (p >= end)
			return end;

		for (i = 0; i < 3; i++) {
			uint32_t tmp = *state << 8;
			*state = tmp + *(p++);
			if (tmp == 0x100 || p == end)
				return p;
		}

		while (p < end) {
			if (p[-1] > 1) p += 3;
			else if (p[-2]) p += 2;
			else if (p[-3] | (p[-1] - 1)) p++;
			else {
				p++;
				break;
			}
		}

		p = std::min(p, end) - 4;
		*state = AV_RB32(p);

		return p + 4;
	}

	static inline void* ts_memmem(const void *haystack, size_t haystack_len,
		const void *needle, size_t needle_len)
	{
		const char *begin = (const char *)haystack;
		const char *last_possible = begin + haystack_len - needle_len;
		const char *tail = (const char *)needle;
		char point;

		/*
		 * The first occurrence of the empty string is deemed to occur at
		 * the beginning of the string.
		 */
		if (needle_len == 0)
			return (void *)begin;

		/*
		 * Sanity check, otherwise the loop might search through the whole
		 * memory.
		 */
		if (haystack_len < needle_len)
			return NULL;

		point = *tail++;
		for (; begin <= last_possible; begin++) {
			if (*begin == point && !memcmp(begin + 1, tail, needle_len - 1))
				return (void *)begin;
		}

		return NULL;
	}

	mpegts_parser::mpegts_parser()
	{
		m_stream_types[0x01] = "MPEG2VIDEO|ISO/IEC 11172-2 Video";	// v
		m_stream_types[0x02] = "MPEG2VIDEO|ISO/IEC 13818-2 Video";	// v
		m_stream_types[0x03] = "MP3|ISO/IEC 11172-3 Audio";
		m_stream_types[0x04] = "MP3|ISO/IEC 13818-3 Audio";
		m_stream_types[0x05] = "ISO/IEC 13818-1 PRIVATE SECTION";
		m_stream_types[0x06] = "ISO/IEC 13818-1 PES";
		m_stream_types[0x07] = "ISO/IEC 13522 MHEG";
		m_stream_types[0x08] = "ISO/IEC 13818-1 Annex A DSM-CC";
		m_stream_types[0x09] = "ITU-T Rec.H.222.1";
		m_stream_types[0x0a] = "ISO/IEC 13818-6 type A";
		m_stream_types[0x0b] = "ISO/IEC 13818-6 type B";
		m_stream_types[0x0c] = "ISO/IEC 13818-6 type C";
		m_stream_types[0x0d] = "ISO/IEC 13818-6 type D";
		m_stream_types[0x0e] = "ISO/IEC 13818-1 AUXILIARY";
		m_stream_types[0x0f] = "AAC"; // AAC|ISO/IEC 13818-7 Audio with ADTS transport syntax
		m_stream_types[0x10] = "MPEG4|ISO/IEC 14496-2 Visual";			// v
		m_stream_types[0x11] = "LATM|ISO/IEC 14496-3 Audio with the LATM transport syntax as defined in ISO/IEC 14496-3 / AMD 1";
		m_stream_types[0x12] = "ISO/IEC 14496-1 SL-packetized stream or FlexMux stream carried in PES packets";
		m_stream_types[0x13] = "ISO/IEC 14496-1 SL-packetized stream or FlexMux stream carried in ISO/IEC14496_sections";
		m_stream_types[0x14] = "ISO/IEC 13818-6 Synchronized Download Protocol";
		m_stream_types[0x1b] = "H264";			// v
		m_stream_types[0x20] = "H264";			// v
		m_stream_types[0x24] = "HEVC";			// v
		m_stream_types[0x42] = "CAVS";			// v
		m_stream_types[0xd1] = "DIRAC";			// v
		m_stream_types[0xea] = "VC1";			// v

		m_stream_types[0x80] = "PCM_BLURAY";
		m_stream_types[0x81] = "AC3|DOLBY_AC3_AUDIO";
		m_stream_types[0x82] = "DTS";
		m_stream_types[0x83] = "TRUEHD";
		m_stream_types[0x84] = "EAC3";
		m_stream_types[0x85] = "DTS";
		m_stream_types[0x86] = "DTS";
		m_stream_types[0x8a] = "DTS";
		m_stream_types[0xa1] = "EAC3";
		m_stream_types[0xa2] = "DTS";
		m_stream_types[0x90] = "HDMV_PGS_SUBTITLE";

		m_streams.resize(0x2000, 0);
		m_pcr_pid = -1;
		m_has_pat = false;
		m_matadata.resize(188 * 2);
		m_cc_pids.resize(0x2000, -1);

		m_packet_count = -1;
		m_pcr_packet_count = 0;
		m_pat_count = 0;
		m_pmt_count = 0;
		m_pmt_pid = 4095;
		m_first_pcr = 0;
		m_total_bytes = 0;
	}

	mpegts_parser::~mpegts_parser()
	{

	}

	inline bool mpegts_parser::do_internal_parser(const uint8_t* parse_ptr, mpegts_info& info, bool check_crc/* = false*/)
	{
		// 如果不是同步字节则跳过.
		if (*parse_ptr != 0x47)
			return false;

		// 解析PID等mpegts头信息.
		const uint8_t* base_ptr = parse_ptr;
		uint16_t PID = ((parse_ptr[1] & 0x1f) << 8) | parse_ptr[2];
		bool payload_unit_start_indicator = !!(parse_ptr[1] & 0x40);
		bool has_payload = !!(parse_ptr[3] & 0x10);
		bool has_adaptation = !!(parse_ptr[3] & 0x20);	// adaptation_field_control
		info.cc_ = parse_ptr[3] & 0x0f;
		info.start_ = payload_unit_start_indicator;
		info.pid_ = PID;
		info.stream_type_ = m_streams[PID];

		// 跳过这些专用数据包.
		if (PID == 0x0001 || PID == 0x0002 || PID == 0x0010 || PID == 0x0011 ||
			PID == 0x0012 || PID == 0x0013 || PID == 0x0014 || PID == 0x001E ||
			PID == 0x001F || PID == 0x1FFF)
		{
			info.type_ = mpegts_info::reserve;
			if (PID == 0x1FFF)
				info.type_ = mpegts_info::nullpkt;
			return true;
		}

		if (PID == 0)
		{
			if (has_adaptation)
			{
				unsigned char adaptation_field_length = *(uint8_t*)&parse_ptr[4];
				parse_ptr += adaptation_field_length + 1;
			}
			parse_ptr += 4;
			if (payload_unit_start_indicator)
			{
				uint8_t pointer_field = *parse_ptr;
				if (pointer_field > 0)
					parse_ptr += pointer_field;
				parse_ptr += 1;
			}

			auto section_ptr = parse_ptr;
			parse_ptr += 1;	// sikp table id, table id == 0.
			int section_length = ((parse_ptr[0] & 0x0f) << 8) | parse_ptr[1];
			parse_ptr += 2;
			// auto program_number = (parse_ptr[0] << 8) | parse_ptr[1];
			parse_ptr += 5;
			if (section_length >= TS_SIZE - TS_HEADER_SIZE)
			{
				std::cerr << "parse section_length error, section_length = " << section_length << std::endl;
				return false;
			}
			// skip following
			// 9 = program_number(16) + reserved(2) +
			//	   version_number(5) + current_next_indicator(1) +
			//     section_number(8) + last_section_number(8) + CRC_32(4bytes)
			for (int n = 0; n < section_length - 9; n += 4)
			{
				uint16_t program_number = (parse_ptr[0] << 8) | parse_ptr[1];
				if (program_number < 0)
					break;
				parse_ptr += 2;
				uint16_t pmt_id = ((parse_ptr[0] & 0x1f) << 8) | parse_ptr[1];
				BOOST_ASSERT(pmt_id <= 0x1fff);
				if (pmt_id == PID)
					break;
				parse_ptr += 2;
				if (program_number == 0)
					continue;
				m_has_pat = true;
				m_pmt_pids.set(pmt_id);
			}

			if (check_crc)
			{
				// CRC32.
				info.crc_ = AV_RB32(parse_ptr);
				auto crc = crc32(section_ptr, parse_ptr - section_ptr);
				if (crc != info.crc_)
				{
					std::cerr << "parse PAT section crc32 error, crc = " << crc
						<< ", crc in data = " << info.crc_ << std::endl;
				}
			}

			// 保存PAT数据包.
			if (m_matadata.size() == 0)
				m_matadata.resize(188 * 2);
			std::memcpy(&m_matadata[0], base_ptr, 188);

			// 设置返回信息.
			info.type_ = mpegts_info::pat;

			return true;
		}

		if (m_pmt_pids[PID] && m_has_pat)
		{
			if (has_adaptation)
			{
				unsigned char adaptation_field_length = *(uint8_t*)&parse_ptr[4];
				parse_ptr += adaptation_field_length + 1;
			}

			parse_ptr += 4;
			if (payload_unit_start_indicator)
			{
				uint8_t pointer_field = *parse_ptr;		// pointer_field 只有当ts包是一个PSI时, 才会包含.
				if (pointer_field > 0)
					parse_ptr += pointer_field;
				parse_ptr += 1;
			}

			auto section_ptr = parse_ptr;
			uint8_t table_id = *parse_ptr;	// table id == 2.
			parse_ptr++;
			uint16_t section_length = ((parse_ptr[0] & 0x0f) << 8) | parse_ptr[1];
			parse_ptr += 2;		// section_syntax_indicator(1) + '0'(1) + reserved(2) + section_length(12)
			auto transport_stream_id = (parse_ptr[0] << 8) | parse_ptr[1];
			parse_ptr += 2;		// transport_stream_id.
			parse_ptr++;		// reserved, version_number, current_next_indicator.
			parse_ptr++;		// section_number.
			parse_ptr++;		// last_section_number.
			m_pcr_pid = ((parse_ptr[0] & 0x1f) << 8) + parse_ptr[1];
			parse_ptr += 2;		// reserved(3) + PCR_PID(13).
			uint16_t program_info_length = ((parse_ptr[0] & 0x0f) << 8) | parse_ptr[1];
			parse_ptr += 2;		// program_info_length(16).
			parse_ptr += program_info_length; // skip program_info descriptor.

			// 9 = transport_stream_id(16) + reserved(2) + version_number(5) +
			//     current_next_indicator(1) + section_number(8) +
			//     last_section_number(8) + reserved(3) + PCR_PID(13) +
			//     reserved(4) + program_info_length(12)
			// 4 = crc32(4)
			int N1 = section_length - (9 + 4 + program_info_length);
			if (N1 >= TS_SIZE - TS_HEADER_SIZE || N1 < 0)
			{
				std::cerr << "parse N1 error, N1 = " << N1 << std::endl;
				return false;
			}

			for (int n = 0; n < N1;)
			{
				uint8_t stream_type = *parse_ptr;
				if (m_stream_types.find(stream_type) != m_stream_types.end())
				{
					parse_ptr++;					// stream_type.
					uint16_t elementary_PID = ((parse_ptr[0] & 0x1F) << 8) | parse_ptr[1];
					BOOST_ASSERT(elementary_PID <= 0x1fff);
					parse_ptr += 2;					// reserved + elementary_PID.
					uint16_t ES_info_length = ((parse_ptr[0] & 0x0F) << 8) | parse_ptr[1];
					parse_ptr += 2;					// reserved + ES_info_length.
					parse_ptr += ES_info_length;	// skip ES_info descriptor.
					n += (5 + ES_info_length);

					// 记录音频和视频的PID.
					m_streams[elementary_PID] = stream_type;
					if (stream_type == 0x1b || stream_type == 0x20)
						m_streams[elementary_PID] = video_h264;
					else if (stream_type == 0x24)
						m_streams[elementary_PID] = video_hevc;

					if (stream_type == 0x01 || stream_type == 0x02 ||
						stream_type == 0x1b || stream_type == 0x20 ||
						stream_type == 0x10 || stream_type == 0x24 ||
						stream_type == 0x42 || stream_type == 0xd1 ||
						stream_type == 0xea)
					{
						m_video_elementary_PIDs.set(elementary_PID);
					}
					else if (
						stream_type == 0x03 || stream_type == 0x04 ||
						stream_type == 0x0f || stream_type == 0x11 ||
						stream_type == 0x80 || stream_type == 0x81 ||
						stream_type == 0x82 || stream_type == 0x83 ||
						stream_type == 0x84 || stream_type == 0x85 ||
						stream_type == 0x86 || stream_type == 0x8a ||
						stream_type == 0xa1 || stream_type == 0xa2 ||
						stream_type == 0x90)
					{
						m_audio_elementary_PIDs.set(elementary_PID);
					}
					else
					{
						std::cerr << "parse mpegts, unexpected stream type, type = "
							<< (int)stream_type << std::endl;
					}
				}
				else
				{
					std::cerr << "parse stream type error, N1 = "
						<< N1 << ", stream_type = " << (int)stream_type << std::endl;
					return false;
				}
			}

			if (check_crc)
			{
				// CRC32.
				info.crc_ = AV_RB32(parse_ptr);
				auto crc = crc32(section_ptr, parse_ptr - section_ptr);
				if (crc != info.crc_)
				{
					std::cerr << "parse PMT section crc32 error, crc = " << crc
						<< ", crc in data = " << info.crc_ << std::endl;
				}
			}

			// 保存PMT数据包到matadata中.
			std::memcpy(&m_matadata[188], base_ptr, 188);

			info.type_ = mpegts_info::pmt;

			return true;
		}

		if (has_payload)
		{
			info.type_ = mpegts_info::data;
			uint8_t* payload = nullptr;
			if (!has_adaptation)
				payload = (uint8_t*)parse_ptr + TS_HEADER_SIZE;
			else
				payload = (uint8_t*)parse_ptr + TS_HEADER_SIZE + 1 + parse_ptr[4];
			int pes_headerlength = 0;
			if (payload_unit_start_indicator)
			{
				pes_headerlength = payload[8];
				int pes_length = (payload[4] << 8) | payload[5];
				int stream_id = payload[3];
				if (stream_id == 190)
				{
					std::cout << "1011 1110\n";
				}
				bool has_pts = !!(payload[7] & 0x80);
				bool has_dts = (payload[7] & 0xc0) == 0xc0;
				uint64_t pts = 0, dts = 0;

				if (has_pts)
				{
					pts = (((uint64_t)payload[9] & 0xe)) << 29 | (payload[10] << 22) |
						((payload[11] & 0xfe) << 14) | (payload[12] << 7) |
						((payload[13] & 0xfe) >> 1);

					info.pts_ = pts;
				}

				if (has_dts)
				{
					dts = (((uint64_t)payload[14] & 0xe)) << 29 | (payload[15] << 22) |
						((payload[16] & 0xfe) << 14) | (payload[17] << 7) |
						((payload[18] & 0xfe) >> 1);

					info.dts_ = dts;
				}
				else
				{
					info.dts_ = info.pts_;
				}
				payload += PES_HEADER_SIZE + PES_HEADER_OPTIONAL_SIZE + pes_headerlength;
			}

			std::size_t payload_size = 188 - (payload - parse_ptr);
			if (payload_size > TS_SIZE - TS_HEADER_SIZE)
			{
				std::cerr << "parse payload_size error, payload_size = " << payload_size << std::endl;
				payload_size = 0;
			}

			info.payload_begin_ = payload;
			info.payload_end_ = payload + payload_size;
		}

		if (m_pcr_pid == PID)
		{
			if (/*(parse_ptr[3] & 0x20)*/ has_adaptation && // adaptation.
				(parse_ptr[5] & 0x10) &&
				(parse_ptr[4] >= 7))
			{
				// PCR is 33 bits.
				info.pcr_ = ((int64_t)parse_ptr[6] << 25) |
					((int64_t)parse_ptr[7] << 17) |
					((int64_t)parse_ptr[8] << 9) |
					((int64_t)parse_ptr[9] << 1) |
					(((int64_t)parse_ptr[10] & 0x80) >> 7);
				int64_t pcr_ext =
					(parse_ptr[10] & 0x01) << 8 |
					(parse_ptr[11]);
				info.pcr_ = (info.pcr_ * 300 + pcr_ext) / 27000;
			}
		}

		// 是否为音视频流.
		info.is_video_ = m_video_elementary_PIDs[PID];
		info.is_audio_ = m_audio_elementary_PIDs[PID];

		// 视频流中是否有这个PID的流信息.
		if (!info.is_video_ && !info.is_audio_)
			return true;

		// 如果是起始包, 则置类型为0, 以查询这个帧的类型.
		if (payload_unit_start_indicator)
			m_type_pids[PID] = 0;

// #define DUMP_VIDEO_PAYLOAD
#if defined(DUMP_VIDEO_PAYLOAD)
		if (info.is_video_ && has_payload)
		{
			static FILE* fp = fopen("dump_video_payload.dat", "w+b");
			fwrite(info.payload_begin_, 1, info.payload_end_ - info.payload_begin_, fp);
			fflush(fp);
		}
#endif
// #define DUMP_AUDIO_PAYLOAD
#if defined(DUMP_AUDIO_PAYLOAD)
		if (info.is_audio_ && has_payload)
		{
			static FILE* fp = fopen("dump_audio_payload.dat", "w+b");
			fwrite(info.payload_begin_, 1, info.payload_end_ - info.payload_begin_, fp);
			fflush(fp);
		}
#endif

		if (has_payload && info.is_video_)
		{
			auto has_found_type = m_type_pids[PID];
			const uint8_t* ptr = info.payload_begin_;
			const uint8_t* end = info.payload_end_;

			switch (info.stream_type_)
			{
				case video_h264:
					if (!has_found_type)
						do_parse_h264(ptr, end, info);
					break;
				case video_hevc:
					if (!has_found_type)
						do_parse_hevc(ptr, end, info);
					break;
				case video_mpeg1:
				case video_mpeg2:
					do_parse_mpeg2(ptr, end, info);
			}
		}

		return true;
	}

	inline void mpegts_parser::do_parse_h264(const uint8_t* ptr, const uint8_t* end, mpegts_info& info)
	{
		uint32_t state = -1;
		int nalu_type;
		while (ptr < end)
		{
			ptr = find_start_code(ptr, end, &state);
			if ((state & 0xFFFFFF00) != 0x100)
				break;
			nalu_type = state & 0x1F;
			enum {
				NAL_UNIT_TYPE_UNKNOWN = 0,
				NAL_UNIT_TYPE_SLICE = 1,
				NAL_UNIT_TYPE_DPA = 2,
				NAL_UNIT_TYPE_DPB = 3,
				NAL_UNIT_TYPE_DPC = 4,
				NAL_UNIT_TYPE_IDR = 5,
				NAL_UNIT_TYPE_SEI = 6,
				NAL_UNIT_TYPE_SPS = 7,
				NAL_UNIT_TYPE_PPS = 8,
				NAL_UNIT_TYPE_AUD = 9,
				NAL_UNIT_TYPE_END_SEQUENCE = 10,
				NAL_UNIT_TYPE_END_STREAM = 11,
			};
			if (nalu_type == NAL_UNIT_TYPE_SLICE || nalu_type <= NAL_UNIT_TYPE_IDR)
			{
				m_type_pids[info.pid_] = 1;
				if (nalu_type == NAL_UNIT_TYPE_IDR)
				{
					info.type_ = mpegts_info::idr;
					info.pict_type_ = av_picture_type_i;
				}
#ifndef DISABLE_PARSE_PICT_TYPE
				else
				{
					bitstream bs(ptr, static_cast<int>(end - ptr));
					bs.read_ue(); // skip first_mb_in_slice.
					auto slice_type = bs.read_ue();
					info.pict_type_ = ts_h264_golomb_to_pict_type[slice_type % 5];
				}
#endif // DISABLE_PARSE_PICT_TYPE
				break;
			}
		}
	}

	inline void mpegts_parser::do_parse_hevc(const uint8_t* ptr, const uint8_t* end, mpegts_info& info)
	{
		uint32_t state = -1;
		int nalu_type;

		while (ptr < end)
		{
			ptr = find_start_code(ptr, end, &state);
			if (--ptr + 2 >= end)
				break;
			nalu_type = (*ptr >> 1) & 0x3f;
			if (nalu_type >= 16 && nalu_type <= 23)
			{
				info.type_ = mpegts_info::idr;
				info.pict_type_ = av_picture_type_i;
				m_type_pids[info.pid_] = 1;
				break;
			}
		}
	}

	inline void mpegts_parser::do_parse_mpeg2(const uint8_t* ptr, const uint8_t* end, mpegts_info& info)
	{
		while (ptr < end)
		{
			ptr = (uint8_t*)ts_memmem(ptr, end - ptr, "\000\000\001", 3);
			if (!ptr)
				break;

			if (ptr[3] == 0x00)
			{
				info.pict_type_ = (ptr[5] & 0x38) >> 3;

				if (info.pict_type_ == av_picture_type_i)
				{
					info.type_ = mpegts_info::idr;
					info.start_ = true;
					break;
				}

				if (info.pict_type_ == av_picture_type_p || info.pict_type_ == av_picture_type_b)
				{
					info.start_ = true;
					break;
				}
			}

			ptr += 3;
		}
	}

	bool mpegts_parser::do_parser(const uint8_t* parse_ptr, mpegts_info& info, bool check_crc/* = false*/)
	{
		bool ret = do_internal_parser(parse_ptr, info);

#if CONTINUITY_CHECK
		// 检查cc的连续性.
		if (ret)
		{
			if (info.pid_ < 0 || info.pid_ >= 0x2000)
				return ret;
			if (info.cc_ != m_cc_pids[info.pid_] + 1)
			{
				if (m_cc_pids[info.pid_] == -1) // first set cc.
					m_cc_pids[info.pid_] = info.cc_;
				else
				{
					if (m_cc_pids[info.pid_] != 0xf || info.cc_ != 0x0)
					{
						std::cerr << "Continuity check failed for pid " << info.pid_ <<
							" expected " << (int)m_cc_pids[info.pid_] + 1 <<
							", got " << info.cc_ << std::endl;
					}
				}
			}
			m_cc_pids[info.pid_] = info.cc_;
		}
#endif

		return ret;
	}

	std::vector<uint8_t>& mpegts_parser::matadata()
	{
		return m_matadata;
	}

	uint8_t mpegts_parser::stream_type(uint16_t pid) const
	{
		return m_streams[pid];
	}

	uint16_t mpegts_parser::stream_type(const std::string& name) const
	{
		for (const auto& t : m_stream_types)
		{
			if (t.second == name)
				return t.first;
		}

		return 0;
	}

	bool mpegts_parser::init_streams(const std::vector<stream_info>& streams)
	{
		for (auto& s : streams)
		{
			auto& p = m_mpegts[s.pid_];
			p.pid_ = s.pid_;
			if (p.pid_ == 0)
			{
				return false;
			}

			p.is_audio_ = s.type_ == stream_info::stream_audio;
			p.is_video_ = s.type_ == stream_info::stream_video;

			if (!p.is_audio_ && !p.is_video_)
			{
				return false;
			}

			p.stream_type_ = s.stream_type_;

			if (p.stream_type_ == 0x01 ||
				p.stream_type_ == 0x02 ||
				p.stream_type_ == 0x03 ||
				p.stream_type_ == 0x04 ||
				p.stream_type_ == 0x05 ||
				p.stream_type_ == 0x06 ||
				p.stream_type_ == 0x07 ||
				p.stream_type_ == 0x08 ||
				p.stream_type_ == 0x09 ||
				p.stream_type_ == 0x0a ||
				p.stream_type_ == 0x0b ||
				p.stream_type_ == 0x0c ||
				p.stream_type_ == 0x0d ||
				p.stream_type_ == 0x0e ||
				p.stream_type_ == 0x0f ||
				p.stream_type_ == 0x10 ||
				p.stream_type_ == 0x11 ||
				p.stream_type_ == 0x12 ||
				p.stream_type_ == 0x13 ||
				p.stream_type_ == 0x14 ||
				p.stream_type_ == 0x1b ||
				p.stream_type_ == 0x20 ||
				p.stream_type_ == 0x24 ||
				p.stream_type_ == 0x42 ||
				p.stream_type_ == 0xd1 ||
				p.stream_type_ == 0xea)
			{
				// stream type is ok.
			}
			else
			{
				return false;
			}
		}

		return true;
	}

	void mpegts_parser::add_pat(uint8_t* ts)
	{
		// TS HEADER.
		ts_set_pid(ts, 0);
		ts_set_transportpriority(ts);
		ts_set_payload(ts);
		ts_set_unitstart(ts);
		ts_set_cc(ts, m_pat_count++);
		// ts_set_adaptation(ts, 0); // PAT不添加adaptation.

		// TS SECTION.
		auto section = ts_section(ts);
		psi_set_tableid(section, 0);	// PAT的table id=0.
		psi_set_syntax(section);
		// section_size = (pmt_size * program(4bytes)) +
		//                transport_stream_id(16) + reserved(2) +
		//                version_number(5) + current_next_indicator(1) +
		//                section_number(8) + last_section_number(8) +
		//                CRC_32(4bytes)
		auto pmt_size = 1; // 只实现生成一个PMT的PAT, 多个PMT暂时不考虑.
		auto section_size = pmt_size * 4 + 5 + 4;
		psi_set_length(section, static_cast<uint16_t>(section_size));
		psi_set_number(section, 1); // program_number.

		psi_set_version(section, 1);
		psi_set_current(section);
		psi_set_section(section, 0);
		psi_set_lastsection(section, 0);

		for (int i = 0; i < pmt_size; i++)
		{
			auto npmt = pat_get_program(section, i);
			patn_set_program(npmt, i + 1);
			patn_set_pid(npmt, m_pmt_pid);
		}

		// crc32.
		psi_set_crc(section);
		psi_set_end(section);
	}

	void mpegts_parser::add_pmt(uint8_t* ts)
	{
		// TS HEADER.
		ts_set_pid(ts, m_pmt_pid);
		ts_set_transportpriority(ts);
		ts_set_payload(ts);
		ts_set_unitstart(ts);
		ts_set_cc(ts, m_pmt_count++);
		// ts_set_adaptation(ts, 0); // PMT不添加adaptation.

		// TS SECTION.
		auto section = ts_section(ts);
		psi_set_tableid(section, 2); // PMT的table id=2.
		psi_set_syntax(section);
		// section_size = (stream_size * program(5bytes)) +
		//                transport_stream_id(16) + reserved(2) +
		//                version_number(5) + current_next_indicator(1) +
		//                section_number(8) + last_section_number(8) +
		//                reserved(3) + PCR_PID(13) + reserved(4) +
		//                program_info_length(12) + program_info_length + CRC_32(4bytes)
		auto stream_size = m_mpegts.size();
		auto section_size = stream_size * 5 + 9 + 4;
		psi_set_length(section, static_cast<uint16_t>(section_size));
		psi_set_number(section, 1);	// transport_stream_id.
		psi_set_version(section, 1);
		psi_set_current(section);
		psi_set_section(section, 0);
		psi_set_lastsection(section, 0);

		auto iter = m_mpegts.begin();
		pmt_set_pcrpid(section, iter->first);	// 不使用独立的pcr包, 默认第1个流的pid带pcr信息.
		pmt_set_desclength(section, 0);

		for (int i = 0; i < stream_size; i++)
		{
			auto& info = iter->second;
			iter++;
			auto pmtn = pmt_get_es(section, i);
			pmtn_init(pmtn);
			pmtn_set_streamtype(pmtn, info.stream_type_);
			if (info.is_video_)
				pmt_set_pcrpid(section, info.pid_);	// 更新为视频流的pid带pcr信息.
			pmtn_set_pid(pmtn, info.pid_);
		}

		// crc32.
		psi_set_crc(section);
		psi_set_end(section);
	}

	bool mpegts_parser::mux_stream(const mpegts_info& info)
	{
		// 查询是否在编码容器当中.
		auto found = m_mpegts.find(info.pid_);
		if (found == m_mpegts.end())
		{
			return false;
		}

		auto& cur_stream = found->second;

		bool write_pat_pmt = false;
		bool write_pcr = true;
		bool only_audio = false;

		// 判断是否添加pat pmt, 如果只有音频, 那就按pcr一样
		// 的规则插入pat pmt包.
		auto stream_size = m_mpegts.size();
		if (stream_size == 1 && info.is_audio_)
			only_audio = true;

		// 如果是开始编码，那就必须插入pcr, pat, pmt.
		if (m_packet_count == -1)
		{
			write_pat_pmt = true;
			write_pcr = true;
			m_packet_count = 0;
		}

		// 如果有音视频, 且此次写入的是音频, 则不写入pcr.
		if (info.is_audio_ && stream_size > 1)
		{
			write_pcr = false;
		}

		// 插入pat/pmt包.
		if (write_pat_pmt)
		{
			// 添加PAT包.
			auto pat_data = m_mpegts_data.prepare(188);
			add_pat(pat_data);
			m_mpegts_data.commit(188);

			// 添加PMT包.
			auto pmt_data = m_mpegts_data.prepare(188);
			add_pmt(pmt_data);
			m_mpegts_data.commit(188);

			// 更新字节数.
			m_total_bytes += (188 * 2);
			m_packet_count += 2;
		}

		bool unitstart = true;
		const uint8_t* begin = info.payload_begin_;
		const uint8_t* end = info.payload_end_;
		for (; begin != end;)
		{
			// 以188为1个TS包写入流.
			auto ts = m_mpegts_data.prepare(188);

			// TS HEADER.
			ts_set_pid(ts, 0);
			ts_set_transportpriority(ts);
			ts_set_payload(ts);
			ts_set_cc(ts, cur_stream.cc_++);
			if (unitstart)
				ts_set_unitstart(ts);	// 设置为起始包.

			if (write_pcr)
			{
				write_pcr = false;

				ts_set_adaptation(ts, 7);	// 写入pcr信息.
				// auto afc = ts_adaptation_field(ts);
				// *afc = 0b00100000;		// only PCR_flag.
				tsaf_set_discontinuity(ts);
				tsaf_set_randomaccess(ts);
				tsaf_set_streampriority(ts);

				int64_t pcr = 0;
				if (info.dts_ != -1)
					pcr = info.dts_ * 300;
				else
					pcr = av_rescale(m_total_bytes + 11, 8 * PCR_TIME_BASE, 1) + m_first_pcr;
				int64_t pcr_low = pcr % 300, pcr_high = pcr / 300;
				tsaf_set_pcr(ts, pcr_high);
				tsaf_set_pcrext(ts, pcr_low);
			}

			// 数据写入位置.
			auto payload = ts_payload(ts);

			// 如果是关键帧, 则开始写入PES.
			if (info.pict_type_ == av_picture_type_i && unitstart)
			{
				int stream_id = -1;
				if (info.is_video_)
				{
					if (cur_stream.stream_type_ == video_dirac)
						stream_id = 0xfd;
					else
						stream_id = 0xe0;
				}
				else
				{
					if (cur_stream.stream_type_ == audio_mpeg2||
						cur_stream.stream_type_ == audio_mpeg1||
						cur_stream.stream_type_ == audio_aac||
						cur_stream.stream_type_ == audio_aac_latm)
						stream_id = 0xc0;
					else if (cur_stream.stream_type_ == audio_ac3)
						stream_id = 0xfd;
					else
						stream_id = 0xfc;
				}

				auto pes = ts_payload(ts);
				pes_init(pes);
				int pes_header_length = 0;
				if (info.dts_ != -1)
					pes_header_length += 5;
				if (info.pts_ != -1)
					pes_header_length += 5;
				pes_set_streamid(pes, stream_id);
				// total pes length = PES_HEADER_SIZE + PES_HEADER_OPTIONAL_SIZE
				//					  + pes_header_length + pes_length.
				pes_set_length(pes, 0);
				pes_set_headerlength(pes, pes_header_length);
				if (info.dts_ != -1)
					pes_set_dts(pes, info.dts_);
				if (info.pts_ != -1)
					pes_set_pts(pes, info.pts_);
				payload = pes_payload(pes);
			}

			auto header_len = payload - ts;
			auto payload_size = end - begin;
			auto len = TS_SIZE - header_len;
			if (len > payload_size)
				len = payload_size;
			auto stuffing_len = TS_SIZE - header_len - len;
			if (stuffing_len > 0)
			{
				if (ts_has_adaptation(ts))
				{
					// stuffing already present: increase its size.
					auto afc_len = ts_get_adaptation_length(ts);
					memmove(ts + 4 + afc_len + stuffing_len,
						ts + 4 + afc_len,
						header_len - (4 + afc_len));
					ts[4] += stuffing_len;
					memset(ts + 4 + afc_len, 0xff, stuffing_len);
				}
				else
				{
					// add stuffing.
					memmove(ts + 4 + stuffing_len, ts + 4, header_len - 4);
					ts[3] |= 0x20;
					ts[4] = stuffing_len - 1;
					if (stuffing_len >= 2)
					{
						ts[5] = 0x00;
						memset(ts + 6, 0xff, stuffing_len - 2);
					}
				}
			}

			if (payload_size == len)
			{
				memcpy(ts + TS_SIZE - len, begin, len - 1);
				ts[TS_SIZE - 1] = 0xff;
			}
			else
			{
				memcpy(ts + TS_SIZE - len, begin, len);
			}

			begin += len;
			m_mpegts_data.commit(188);
			m_packet_count++;
			unitstart = false;
		}

		return false;
	}

	int mpegts_parser::mpegts_size() const
	{
		return m_mpegts_data.size();
	}

	void mpegts_parser::fetch_mpegts(uint8_t* data, int size)
	{
		memcpy(data, m_mpegts_data.data(), size);
		m_mpegts_data.consume(size);
	}

	std::string mpegts_parser::stream_name(uint16_t pid) const
	{
		auto found = m_stream_types.find(m_streams[pid]);
		if (found != m_stream_types.end())
			return std::string(found->second);
		return "";
	}

	byte_streambuf::byte_streambuf()
	{
		clear();
	}

	byte_streambuf::byte_streambuf(byte_streambuf&& rhs)
		: m_buffer(std::move(rhs.m_buffer))
	{
		setg(rhs.m_get_first, rhs.m_get_next, rhs.m_get_last);
		setp(rhs.m_put_first, rhs.m_put_next, rhs.m_put_last);
	}

	byte_streambuf::~byte_streambuf()
	{}

	void byte_streambuf::clear()
	{
		m_buffer.swap(std::vector<uint8_t>());

		m_max_size = std::numeric_limits<std::size_t>::max();
		std::size_t pend = (std::min<std::size_t>)(m_max_size, buffer_delta);
		m_buffer.resize((std::max<std::size_t>)(pend, 1));

		setg(&m_buffer[0], &m_buffer[0], &m_buffer[0]);
		setp(&m_buffer[0], &m_buffer[0] + pend);
	}

	void byte_streambuf::shrink_to_fit()
	{
		// Get current stream positions as offsets.
		std::size_t gnext = gptr() - &m_buffer[0];
		std::size_t pnext = pptr() - &m_buffer[0];
		std::size_t pend = epptr() - &m_buffer[0];

		if ((pend - pnext) > (pnext - gnext))
		{
			// Shift existing contents of get area to start of buffer.
			if (gnext > 0)
			{
				pnext -= gnext;
				std::memmove(&m_buffer[0], &m_buffer[0] + gnext, pnext);
			}

			m_buffer.resize(pnext);
			m_buffer.shrink_to_fit();

			// Update stream positions.
			pend = pnext;
			setg(&m_buffer[0], &m_buffer[0], &m_buffer[0] + pnext);
			setp(&m_buffer[0] + pnext, &m_buffer[0] + pend);
		}
	}

	size_t byte_streambuf::size() const noexcept
	{
		return pptr() - gptr();
	}

	size_t byte_streambuf::capacity() const noexcept
	{
		return m_buffer.capacity();
	}

	const uint8_t* byte_streambuf::data() const noexcept
	{
		return gptr();
	}

	uint8_t* byte_streambuf::prepare(size_t n)
	{
		reserve(n);
		return pptr();
	}

	void byte_streambuf::commit(size_t n)
	{
		if (pptr() + n > epptr())
		{
			//	n = epptr() - pptr();
			std::length_error ex("byte_streambuf commit too long");
			throw ex;
		}

		m_put_next += n;
		setg(eback(), gptr(), pptr());
	}

	void byte_streambuf::consume(size_t n)
	{
		if (egptr() < pptr())
			setg(&m_buffer[0], gptr(), pptr());
		if (gptr() + n > pptr())
		{
			// n = pptr() - gptr();
			std::length_error ex("byte_streambuf consume too long");
			throw ex;
		}

		m_get_next += n;
	}

	void byte_streambuf::reserve(std::size_t n)
	{
		// Get current stream positions as offsets.
		std::size_t gnext = gptr() - &m_buffer[0];
		std::size_t pnext = pptr() - &m_buffer[0];
		std::size_t pend = epptr() - &m_buffer[0];

		// Check if there is already enough space in the put area.
		if (n <= pend - pnext)
		{
			return;
		}

		// Shift existing contents of get area to start of buffer.
		if (gnext > 0)
		{
			pnext -= gnext;
			std::memmove(&m_buffer[0], &m_buffer[0] + gnext, pnext);
		}

		// Ensure buffer is large enough to hold at least the specified size.
		if (n > pend - pnext)
		{
			if (n <= m_max_size && pnext <= m_max_size - n)
			{
				pend = pnext + n;
				m_buffer.resize((std::max<std::size_t>)(pend, 1));
			}
			else
			{
				std::length_error ex("byte_streambuf too long");
				throw ex;
			}
		}

		// Update stream positions.
		setg(&m_buffer[0], &m_buffer[0], &m_buffer[0] + pnext);
		setp(&m_buffer[0] + pnext, &m_buffer[0] + pend);
	}

	void byte_streambuf::setg(uint8_t* f, uint8_t* n, uint8_t* l)
	{
		m_get_first = f;
		m_get_next = n;
		m_get_last = l;
	}

	void byte_streambuf::setp(uint8_t* f, uint8_t* l)
	{
		m_put_first = f;
		m_put_next = f;
		m_put_last = l;
	}

	void byte_streambuf::setp(uint8_t* f, uint8_t* n, uint8_t* l)
	{
		m_put_first = f;
		m_put_next = n;
		m_put_last = l;
	}

	uint8_t* byte_streambuf::pptr() const
	{
		return m_put_next;
	}

	uint8_t* byte_streambuf::gptr() const
	{
		return m_get_next;
	}

	uint8_t* byte_streambuf::epptr() const
	{
		return m_put_last;
	}

	uint8_t* byte_streambuf::eback() const
	{
		return m_get_first;
	}

	uint8_t* byte_streambuf::egptr() const
	{
		return m_get_last;
	}

}
