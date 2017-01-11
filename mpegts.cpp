#include "mpegts.hpp"
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

#ifndef AV_RB32
#   define AV_RB32(x)                                \
	(((uint32_t)((const uint8_t*)(x))[0] << 24) |    \
			   (((const uint8_t*)(x))[1] << 16) |    \
			   (((const uint8_t*)(x))[2] <<  8) |    \
				((const uint8_t*)(x))[3])
#endif

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
		m_stream_types[0xa1] = "EAC3";
		m_stream_types[0xa2] = "DTS";
		m_stream_types[0x90] = "HDMV_PGS_SUBTITLE";
		m_stream_types[0x81] = "AC3";
		m_stream_types[0x8a] = "DTS";

		m_streams.resize(0x2000, 0);

		m_has_pat = false;
		m_matadata.resize(188 * 2);
		m_cc_pids.resize(0x2000, -1);
	}

	mpegts_parser::~mpegts_parser()
	{

	}

	inline bool mpegts_parser::do_internal_parser(const uint8_t* parse_ptr, mpegts_info& info)
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

			parse_ptr += 1;	// table id.
			int section_length = ((parse_ptr[0] & 0x0f) << 8) | parse_ptr[1];
			parse_ptr += 7;
			if (section_length >= TS_SIZE - TS_HEADER_SIZE)
			{
				std::cerr << "parse section_length error, section_length = " << section_length << std::endl;
				return false;
			}
			for (int n = 0; n < section_length - 12; n += 4)
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

			uint8_t table_id = *parse_ptr;	// table id.
			parse_ptr++;
			uint16_t section_length = ((parse_ptr[0] & 0x0f) << 8) | parse_ptr[1];
			parse_ptr += 2;
			parse_ptr += 2;		// program_number
			parse_ptr++;		// reserved, version_number, current_next_indicator.
			parse_ptr++;		// section_number.
			parse_ptr++;		// last_section_number.
			uint16_t PCR_PID = ((parse_ptr[0] & 0x1f) << 8) + parse_ptr[1];
			parse_ptr += 2;
			uint16_t program_info_length = ((parse_ptr[0] & 0x0f) << 8) | parse_ptr[1];
			parse_ptr += 2;
			parse_ptr += program_info_length;

			int N1 = section_length - (9 + 4 + program_info_length);
			if (N1 >= TS_SIZE - TS_HEADER_SIZE)
			{
				std::cerr << "parse N1 error, N1 = " << N1 << std::endl;
				return false;
			}

			for (int n = 0; n < N1;)
			{
				uint8_t stream_type = *parse_ptr;
				if (m_stream_types.find(stream_type) != m_stream_types.end())
				{
					parse_ptr++;
					uint16_t elementary_PID = ((parse_ptr[0] & 0x1F) << 8) | parse_ptr[1];
					BOOST_ASSERT(elementary_PID <= 0x1fff);
					parse_ptr += 2;
					uint16_t ES_info_length = ((parse_ptr[0] & 0x0F) << 8) | parse_ptr[1];
					parse_ptr += 2;
					parse_ptr += ES_info_length;
					n += (5 + ES_info_length);

					// 记录音频和视频的PID.
					m_streams[elementary_PID] = stream_type;
					if (stream_type == 0x1b || stream_type == 0x20)
						m_streams[elementary_PID] = video_h264;
					else if (stream_type == 0x24)
						m_streams[elementary_PID] = video_hevc;

					if (stream_type == 0x1b || stream_type == 0x20 ||
						stream_type == 0x01 || stream_type == 0x02 ||
						stream_type == 0x10 || stream_type == 0x24 ||
						stream_type == 0x42 || stream_type == 0xd1 ||
						stream_type == 0xea)
					{
						m_video_elementary_PIDs.set(elementary_PID);
					}
					else if (stream_type == 0x0f || stream_type == 0x03 ||
						stream_type == 0x06 ||
						stream_type == 0x04 || stream_type == 0x80 ||
						stream_type == 0x81 || stream_type == 0x82 ||
						stream_type == 0x83 || stream_type == 0x84 ||
						stream_type == 0x85 || stream_type == 0x86 ||
						stream_type == 0xa1 || stream_type == 0xa2 ||
						stream_type == 0x90 || stream_type == 0x81 ||
						stream_type == 0x8a)
					{
						m_audio_elementary_PIDs.set(elementary_PID);
					}
					else
					{
						std::cerr << "parse mpegts, unexpected stream type, type = " << (int)stream_type << std::endl;
					}
				}
				else
				{
					std::cerr << "parse stream type error, N1 = " << N1 << ", stream_type = " << (int)stream_type << std::endl;
					return false;
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
				info.type_ = mpegts_info::idr;
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
				auto frame_type = (ptr[5] & 0x38) >> 3;

				if (frame_type == 1)
				{
					info.type_ = mpegts_info::idr;
					info.start_ = true;
					break;
				}

				if (frame_type == 2 || frame_type == 3)
				{
					info.start_ = true;
					break;
				}
			}

			ptr += 3;
		}
	}

	bool mpegts_parser::do_parser(const uint8_t* parse_ptr, mpegts_info& info)
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

	std::string mpegts_parser::stream_name(uint16_t pid) const
	{
		auto found = m_stream_types.find(m_streams[pid]);
		if (found != m_stream_types.end())
			return std::string(found->second);
		return "";
	}
}
