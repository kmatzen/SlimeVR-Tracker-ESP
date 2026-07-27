#include "dataset.h"

#include <algorithm>
#include <cctype>
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

// Preserves empty fields, including trailing ones. A getline-based split drops
// trailing empties, which would silently mangle an accelerometer row such as
// "1000,123,-456,16384,,," -- exactly the rows the firmware's raw logger emits.
std::vector<std::string> split(const std::string& s, char sep) {
	std::vector<std::string> out;
	size_t start = 0;
	while (true) {
		size_t p = s.find(sep, start);
		if (p == std::string::npos) {
			out.push_back(trim(s.substr(start)));
			break;
		}
		out.push_back(trim(s.substr(start, p - start)));
		start = p + 1;
	}
	return out;
}

// A field counts as present only if it exists and is non-empty. An empty field
// means "no sample", which is distinct from a sample that reads zero.
bool present(const std::vector<std::string>& row, int idx) {
	return idx >= 0 && idx < static_cast<int>(row.size()) && !row[idx].empty();
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
			} else if (key == "acc_scale") {
				hs >> out.accScale;
			} else if (key == "gyr_scale") {
				hs >> out.gyrScale;
			} else if (key == "mag_scale") {
				hs >> out.magScale;
			} else if (key == "note") {
				std::string rest;
				std::getline(hs, rest);
				out.note = trim(rest);
			}
			continue;
		}

		if (cols.empty()) {
			// A real capture is a serial stream, so ordinary firmware log lines
			// can appear before the header. Wait for the actual column row
			// rather than treating the first non-comment line as one.
			if (t.rfind("t_us", 0) != 0) {
				out.skippedLines++;
				continue;
			}
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

		// Log lines can also appear mid-stream, and a capture may be cut off
		// part-way through a row. Skip anything that is not a well-formed data
		// row and count it, rather than aborting the whole file -- but count it
		// loudly, because silently discarding half a capture would be worse
		// than failing.
		if (t.empty() || !(std::isdigit(static_cast<unsigned char>(t[0])))) {
			out.skippedLines++;
			continue;
		}
		std::vector<std::string> row = split(t, ',');
		if (row.size() < cols.size()) {
			out.skippedLines++;
			continue;
		}

		Sample s;
		s.tUs = static_cast<uint64_t>(std::strtoull(row[idx.t].c_str(), nullptr, 10));

		s.hasAcc = present(row, idx.ax);
		s.hasGyr = present(row, idx.gx);
		s.hasMag = out.hasMag && present(row, idx.mx);

		// Scales default to 1.0, so a file already in physical units is
		// unaffected. A raw capture from the firmware carries its own.
		if (s.hasAcc) {
			s.acc = Vec3{
				field(row, idx.ax) * out.accScale,
				field(row, idx.ay) * out.accScale,
				field(row, idx.az) * out.accScale,
			};
		}
		if (s.hasGyr) {
			s.gyr = Vec3{
				field(row, idx.gx) * out.gyrScale,
				field(row, idx.gy) * out.gyrScale,
				field(row, idx.gz) * out.gyrScale,
			};
		}
		if (s.hasMag) {
			s.mag = Vec3{
				field(row, idx.mx) * out.magScale,
				field(row, idx.my) * out.magScale,
				field(row, idx.mz) * out.magScale,
			};
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
	// Measured between consecutive *gyroscope* rows, not consecutive rows of any
	// kind: in an interleaved capture the latter would report the interleaving
	// period rather than either sensor's actual rate.
	if (out.gyrTs <= 0) {
		std::vector<double> d;
		bool havePrev = false;
		uint64_t prev = 0;
		for (const Sample& s : out.samples) {
			if (!s.hasGyr) {
				continue;
			}
			if (havePrev) {
				d.push_back(static_cast<double>(s.tUs - prev) * 1e-6);
			}
			prev = s.tUs;
			havePrev = true;
		}
		if (!d.empty()) {
			std::sort(d.begin(), d.end());
			out.gyrTs = d[d.size() / 2];
		}
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
		std::snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(s.tUs));
		o << buf;
		if (s.hasAcc) {
			std::snprintf(
				buf,
				sizeof(buf),
				",%.6f,%.6f,%.6f",
				s.acc.x,
				s.acc.y,
				s.acc.z
			);
			o << buf;
		} else {
			o << ",,,";
		}
		if (s.hasGyr) {
			std::snprintf(
				buf,
				sizeof(buf),
				",%.9f,%.9f,%.9f",
				s.gyr.x,
				s.gyr.y,
				s.gyr.z
			);
			o << buf;
		} else {
			o << ",,,";
		}
		if (ds.hasMag && !s.hasMag) {
			o << ",,,";
		} else if (ds.hasMag) {
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
