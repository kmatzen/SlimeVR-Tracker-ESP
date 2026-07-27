#include "dataset.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace fb {
namespace {

std::string trim(const std::string& s) {
	size_t b = s.find_first_not_of(" \t\r\n");
	if (b == std::string::npos) {
		return "";
	}
	size_t e = s.find_last_not_of(" \t\r\n");
	return s.substr(b, e - b + 1);
}

std::vector<std::string> split(const std::string& s, char sep) {
	std::vector<std::string> out;
	std::string cur;
	std::istringstream is(s);
	while (std::getline(is, cur, sep)) {
		out.push_back(trim(cur));
	}
	return out;
}

// Index of `name` in the header, or -1.
int colIndex(const std::vector<std::string>& cols, const std::string& name) {
	for (size_t i = 0; i < cols.size(); i++) {
		if (cols[i] == name) {
			return static_cast<int>(i);
		}
	}
	return -1;
}

double field(const std::vector<std::string>& row, int idx) {
	if (idx < 0 || idx >= static_cast<int>(row.size())) {
		return 0.0;
	}
	return std::strtod(row[idx].c_str(), nullptr);
}

}  // namespace

bool loadDataset(const std::string& path, Dataset& out, std::string& error) {
	std::ifstream in(path);
	if (!in) {
		error = "cannot open " + path;
		return false;
	}

	out = Dataset{};
	out.name = path;

	// Column positions, resolved once when the header row is seen. -1 means the
	// column is absent, which `field()` reads as 0.
	struct Cols {
		int t = -1, ax = -1, ay = -1, az = -1, gx = -1, gy = -1, gz = -1;
		int mx = -1, my = -1, mz = -1;
		int qw = -1, qx = -1, qy = -1, qz = -1;
		int temp = -1;
	} idx;

	std::string line;
	std::vector<std::string> cols;
	int lineNo = 0;

	while (std::getline(in, line)) {
		lineNo++;
		std::string t = trim(line);
		if (t.empty()) {
			continue;
		}

		if (t[0] == '#') {
			// Header directives: "# key value".
			std::istringstream hs(t.substr(1));
			std::string key;
			hs >> key;
			if (key == "gyr_ts") {
				hs >> out.gyrTs;
			} else if (key == "acc_ts") {
				hs >> out.accTs;
			} else if (key == "mag_ts") {
				hs >> out.magTs;
			} else if (key == "note") {
				std::string rest;
				std::getline(hs, rest);
				out.note = trim(rest);
			}
			continue;
		}

		if (cols.empty()) {
			cols = split(t, ',');
			if (colIndex(cols, "t_us") < 0 || colIndex(cols, "ax") < 0
				|| colIndex(cols, "gx") < 0) {
				error = "missing required columns (need at least t_us, ax, gx)";
				return false;
			}
			out.hasMag = colIndex(cols, "mx") >= 0;
			out.hasGroundTruth = colIndex(cols, "qw") >= 0;
			out.hasTemp = colIndex(cols, "temp") >= 0;
			idx.t = colIndex(cols, "t_us");
			idx.ax = colIndex(cols, "ax");
			idx.ay = colIndex(cols, "ay");
			idx.az = colIndex(cols, "az");
			idx.gx = colIndex(cols, "gx");
			idx.gy = colIndex(cols, "gy");
			idx.gz = colIndex(cols, "gz");
			idx.mx = colIndex(cols, "mx");
			idx.my = colIndex(cols, "my");
			idx.mz = colIndex(cols, "mz");
			idx.qw = colIndex(cols, "qw");
			idx.qx = colIndex(cols, "qx");
			idx.qy = colIndex(cols, "qy");
			idx.qz = colIndex(cols, "qz");
			idx.temp = colIndex(cols, "temp");
			continue;
		}

		std::vector<std::string> row = split(t, ',');
		if (row.size() < cols.size()) {
			error = "short row at line " + std::to_string(lineNo);
			return false;
		}

		Sample s;
		s.tUs = static_cast<uint64_t>(std::strtoull(row[idx.t].c_str(), nullptr, 10));
		s.acc = Vec3{field(row, idx.ax), field(row, idx.ay), field(row, idx.az)};
		s.gyr = Vec3{field(row, idx.gx), field(row, idx.gy), field(row, idx.gz)};
		if (out.hasMag) {
			s.mag = Vec3{field(row, idx.mx), field(row, idx.my), field(row, idx.mz)};
		}
		if (out.hasGroundTruth) {
			s.gt = qNorm(Quat{
				field(row, idx.qw),
				field(row, idx.qx),
				field(row, idx.qy),
				field(row, idx.qz),
			});
		}
		if (out.hasTemp) {
			s.temp = field(row, idx.temp);
		}
		out.samples.push_back(s);
	}

	if (out.samples.size() < 2) {
		error = "dataset has fewer than 2 samples";
		return false;
	}

	// Fall back to the median timestamp delta when the header omits the rates.
	if (out.gyrTs <= 0) {
		std::vector<double> d;
		d.reserve(out.samples.size() - 1);
		for (size_t i = 1; i < out.samples.size(); i++) {
			d.push_back(
				static_cast<double>(out.samples[i].tUs - out.samples[i - 1].tUs) * 1e-6
			);
		}
		std::sort(d.begin(), d.end());
		out.gyrTs = d[d.size() / 2];
	}
	if (out.accTs <= 0) {
		out.accTs = out.gyrTs;
	}
	if (out.magTs <= 0) {
		out.magTs = out.gyrTs;
	}

	return true;
}

bool saveDataset(const std::string& path, const Dataset& ds, std::string& error) {
	std::ofstream o(path);
	if (!o) {
		error = "cannot write " + path;
		return false;
	}

	o << "# slimevr-imu-log v1\n";
	o << "# gyr_ts " << ds.gyrTs << "\n";
	o << "# acc_ts " << ds.accTs << "\n";
	if (ds.hasMag) {
		o << "# mag_ts " << ds.magTs << "\n";
	}
	if (!ds.note.empty()) {
		o << "# note " << ds.note << "\n";
	}

	o << "t_us,ax,ay,az,gx,gy,gz";
	if (ds.hasMag) {
		o << ",mx,my,mz";
	}
	if (ds.hasGroundTruth) {
		o << ",qw,qx,qy,qz";
	}
	if (ds.hasTemp) {
		o << ",temp";
	}
	o << "\n";

	char buf[512];
	for (const Sample& s : ds.samples) {
		std::snprintf(
			buf,
			sizeof(buf),
			"%llu,%.6f,%.6f,%.6f,%.9f,%.9f,%.9f",
			static_cast<unsigned long long>(s.tUs),
			s.acc.x,
			s.acc.y,
			s.acc.z,
			s.gyr.x,
			s.gyr.y,
			s.gyr.z
		);
		o << buf;
		if (ds.hasMag) {
			std::snprintf(
				buf,
				sizeof(buf),
				",%.6f,%.6f,%.6f",
				s.mag.x,
				s.mag.y,
				s.mag.z
			);
			o << buf;
		}
		if (ds.hasGroundTruth) {
			std::snprintf(
				buf,
				sizeof(buf),
				",%.9f,%.9f,%.9f,%.9f",
				s.gt.w,
				s.gt.x,
				s.gt.y,
				s.gt.z
			);
			o << buf;
		}
		if (ds.hasTemp) {
			std::snprintf(buf, sizeof(buf), ",%.3f", s.temp);
			o << buf;
		}
		o << "\n";
	}

	return true;
}

}  // namespace fb
