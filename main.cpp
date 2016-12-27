// g++ main.cpp mpegts.cpp -I .

#include "mpegts.hpp"
#include <iostream>
#include <boost/asio/streambuf.hpp>

int main(int argc, char** argv)
{
	FILE* fp = fopen(argv[1], "r+b");
	if (!fp)
		return -1;

	util::mpegts_parser p;
	boost::asio::streambuf buf;
	int vc = 0;
	int sc = 0;
	int offset = 0;
	int64_t pts = 0, dts = 0;
	bool vknown_type = false;
	bool aknown_type = false;

	while (!feof(fp)) {

		if (buf.size() < 188) {
			auto pre = boost::asio::buffer_cast<void*>(buf.prepare(188 * 1000));
			auto sz = fread(pre, 1, 188 * 1000, fp);
			buf.commit(sz);
		}

		while (buf.size() >= 188) {
			const uint8_t* data = boost::asio::buffer_cast<const uint8_t*>(buf.data());
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
				std::cout << "pos=" << offset << std::endl;
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

