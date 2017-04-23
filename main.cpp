#include "mpegts.hpp"
#include <iostream>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int argc, char** argv)
{
	bool show_pcr_time = false;
	bool show_frame_pos = false;
	bool show_frame_pts = false;
	bool show_frame_dts = false;
	bool show_key_frame = false;
	std::string file;

	po::options_description desc("Options");
	desc.add_options()
		("help,h", "Help message.")
		("version", "Current mpegts parser version.")
		("ts", po::value<std::string>(&file), "Specify one input file.")
		("show_pcr_time", po::value<bool>(&show_pcr_time)->default_value(false), "Show pcr time.")
		("show_frame_pos", po::value<bool>(&show_frame_pos)->default_value(false), "Show frame pos.")
		("show_frame_pts", po::value<bool>(&show_frame_pts)->default_value(false), "Show frame pts.")
		("show_frame_dts", po::value<bool>(&show_frame_dts)->default_value(false), "Show frame dts.")
		("show_key_frame", po::value<bool>(&show_key_frame)->default_value(false), "Show key frame.")
		;

	try {
		po::variables_map vm;
		po::store(po::parse_command_line(argc, argv, desc), vm);
		po::notify(vm);

		if (argc < 2 || vm.count("help") || file.empty()) {
			std::cout << desc << "\n";
			return -1;
		}
	}
	catch (std::exception& e) {
		std::cerr << "Exception: " << e.what();
		return -1;
	}

	FILE* fp = fopen(file.c_str(), "r+b");
	if (!fp) {
		std::cerr << "Can't open file " << file << "\n";
		return -1;
	}

	util::mpegts_parser p;
	util::byte_streambuf buf;
	int vc = 0;
	int sc = 0;
	int offset = 0;
	int64_t pts = 0, dts = 0;
	bool vknown_type = false;
	bool aknown_type = false;

	util::stream_info si;
	std::vector<util::stream_info> streams;
	si.pid_ = 40;
	si.type_ = util::stream_info::stream_video;
	si.stream_type_ = p.stream_type(std::string("H264"));
	streams.push_back(si);
	si.pid_ = 50;
	si.type_ = util::stream_info::stream_audio;
	si.stream_type_ = p.stream_type(std::string("AAC"));
	streams.push_back(si);
	p.init_streams(streams);
	util::mpegts_info mi;
	mi.pid_ = 40;
	mi.is_video_ = true;
	mi.is_audio_ = false;
	p.mux_stream(mi);
// 	{
// 		util::mpegts_info info;
// 		auto size = p.mpegts_size();
// 		uint8_t* data = (uint8_t*)malloc(size);
// 		p.fetch_mpegts(data, size);
// 		p.do_parser(data, info);
// 		p.do_parser(data + 188, info);
// 		std::cout << info.pid_;
// 	}
	while (!feof(fp)) {

		if (buf.size() < 188) {
			auto pre = buf.prepare(188 * 1000);
			auto sz = fread(pre, 1, 188 * 1000, fp);
			buf.commit(sz);
		}

		while (buf.size() >= 188) {
			const uint8_t* data = buf.data();
			util::mpegts_info info;
			auto suc = p.do_parser(data, info);
			if (!suc) {
				buf.consume(1);
			} else {
				buf.consume(188);
			}

			if (info.is_video_ && !vknown_type) {
				std::string s = p.stream_name(info.pid_);
				if (!s.empty() && !vknown_type) {
					vknown_type = true;
					std::cout << "pid " << info.pid_ << " stream type: " << s << std::endl;
				}
			}

			if (info.is_audio_ && !aknown_type) {
				std::string s = p.stream_name(info.pid_);
				if (!s.empty() && !aknown_type) {
					aknown_type = true;
					std::cout << "pid " << info.pid_ << " stream type: " << s << std::endl;
				}
			}

			if (info.type_ == util::mpegts_info::idr && info.is_video_)
				vc++;

			if (info.start_ && info.is_video_) {
				sc++;
				if (show_key_frame) {
					if (info.type_ == util::mpegts_info::idr)
						std::cout << "flags=K_" << std::endl;
				}
				if (show_frame_pos)
					std::cout << "pos=" << offset << std::endl;
			}

			if (show_frame_dts) {
				if (info.is_video_ && info.dts_ != -1) {
					std::cout << "dts=" << info.dts_ << std::endl;
				}
			}

			if (show_frame_pts) {
				if (info.is_video_ && info.pts_ != -1) {
					std::cout << "pts=" << info.pts_ << std::endl;
				}
			}

			if (show_pcr_time && info.pcr_ != -1) {
				std::cout << "pcr=" << info.pcr_ << std::endl;
			}

			if (!suc)
				offset += 1;
			else
				offset += 188;
		}
	}
	fclose(fp);
	std::cout << "keyframe count: " << vc << ", frame count " << sc << std::endl;
	return 0;
}

