﻿#include "mpegts.hpp"
#include <iostream>
#include <cstring>
#include <algorithm>
#include <boost/assert.hpp>

namespace util {

#define TS_SIZE						188
#define TS_HEADER_SIZE				4
#define TS_HEADER_SIZE_AF			6
#define TS_HEADER_SIZE_PCR			12
#define PES_HEADER_SIZE				6
#define PES_HEADER_OPTIONAL_SIZE	3

#ifndef AV_RB32
#   define AV_RB32(x)                                \
	(((uint32_t)((const uint8_t*)(x))[0] << 24) |    \
			   (((const uint8_t*)(x))[1] << 16) |    \
			   (((const uint8_t*)(x))[2] <<  8) |    \
				((const uint8_t*)(x))[3])
#endif

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

	static inline bool ts_has_payload(const uint8_t *p_ts)
	{
		return !!(p_ts[3] & 0x10);
	}

	static inline bool ts_has_adaptation(const uint8_t *p_ts)
	{
		return !!(p_ts[3] & 0x20);
	}

	static inline bool ts_get_unitstart(const uint8_t *p_ts)
	{
		return !!(p_ts[1] & 0x40);
	}

	static inline uint8_t ts_get_adaptation(const uint8_t *p_ts)
	{
		return p_ts[4];
	}

	static inline uint8_t *ts_payload(uint8_t *p_ts)
	{
		if (!ts_has_payload(p_ts))
			return p_ts + TS_SIZE;
		if (!ts_has_adaptation(p_ts))
			return p_ts + TS_HEADER_SIZE;
		return p_ts + TS_HEADER_SIZE + 1 + ts_get_adaptation(p_ts);
	}

	static inline uint8_t *ts_section(uint8_t *p_ts)
	{
		if (!ts_get_unitstart(p_ts))
			return ts_payload(p_ts);

		return ts_payload(p_ts) + 1; /* pointer_field */
	}

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
		m_stream_types[0x0f] = "AAC|ISO/IEC 13818-7 Audio with ADTS transport syntax";
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
			parse_ptr += 7;
			if (section_length >= TS_SIZE - TS_HEADER_SIZE)
			{
				std::cerr << "parse section_length error, section_length = " << section_length << std::endl;
				return false;
			}
			// skip following
			// transport_stream_id(16) + reserved(2) +
			// version_number(5) + current_next_indicator(1) +
			// section_number(8) + last_section_number(8) + CRC_32(4)
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
				// CRC32
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
			parse_ptr += 2;		// program_number.
			parse_ptr++;		// reserved, version_number, current_next_indicator.
			parse_ptr++;		// section_number.
			parse_ptr++;		// last_section_number.
			m_pcr_pid = ((parse_ptr[0] & 0x1f) << 8) + parse_ptr[1];
			parse_ptr += 2;		// reserved(3) + PCR_PID(13).
			uint16_t program_info_length = ((parse_ptr[0] & 0x0f) << 8) | parse_ptr[1];
			parse_ptr += 2;		// program_info_length(16).
			parse_ptr += program_info_length; // skip program_info descriptor.

			// 9 = program_number(16) + reserved(2) + version_number(5) +
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
				// CRC32
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

	std::string mpegts_parser::stream_name(uint16_t pid) const
	{
		auto found = m_stream_types.find(m_streams[pid]);
		if (found != m_stream_types.end())
			return std::string(found->second);
		return "";
	}
}
