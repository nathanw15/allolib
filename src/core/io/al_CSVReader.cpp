
#include "al/core/io/al_CSVReader.hpp"

CSVReader::~CSVReader() {
	for (auto row: mData) {
		delete[] row;
	}
	mData.clear();
}

void CSVReader::readFile(std::string fileName) {
	std::ifstream f(fileName);
	if (!f.is_open()) {
		std::cout << "Could not open:" << fileName << std::endl;
		return;
	}

	std::string line;
	size_t rowLength = calculateRowLength();

	getline(f, line); // TODO Process column names
        std::stringstream columnNameStream(line);
        std::string columnName;
        while (std::getline(columnNameStream, columnName, ',')) {
            mColumnNames.push_back(columnName);
        }


//	std::cout << line << std::endl;
	while (getline(f, line)) {
		if (line.size() == 0) {
			continue;
		}
		if ((unsigned long) std::count(line.begin(), line.end(), ',') == mDataTypes.size() - 1) { // Check that we have enough commas
			std::stringstream ss(line);
			char *row = new char[rowLength];
			mData.push_back(row);
			char *data = mData.back();
			int byteCount = 0;
			for(auto type:mDataTypes) {
				std::string field;
				std::getline(ss, field, ',');
				size_t stringLen = std::min(maxStringSize, field.size());
				int32_t intValue;
				double doubleValue;
				bool booleanValue;
				switch (type){
				case STRING:
					std::memcpy(data + byteCount, field.data(), stringLen * sizeof (char));
					byteCount += maxStringSize * sizeof (char);
					break;
				case INTEGER:
					intValue = std::atoi(field.data());
					std::memcpy(data + byteCount, &intValue, sizeof (int32_t));
					byteCount += sizeof (int32_t);
					break;
				case REAL:
					doubleValue = std::atof(field.data());
					std::memcpy(data + byteCount, &doubleValue, sizeof (double));
					byteCount += sizeof (double);
					break;
				case BOOLEAN:
					booleanValue = field == "True" || field == "true";
					std::memcpy(data + byteCount, &booleanValue, sizeof (bool));
					byteCount += sizeof (bool);
					break;
				case NONE:
					break;
				}
			}
		}
	}
	if (f.bad()) {
		std::cout << "Error reading:" << fileName << std::endl;
	}
}

std::vector<double> CSVReader::getColumn(int index) {
	std::vector<double> out;
	int offset = 0;
	for (int i = 0; i < index; i++) {
		switch (mDataTypes[i]){
		case STRING:
			offset += maxStringSize * sizeof (char);
			break;
		case INTEGER:
			offset += sizeof (int32_t);
			break;
		case REAL:
			offset += sizeof (double);
			break;
		case BOOLEAN:
			offset += sizeof (bool);
			break;
		case NONE:
			break;
		}
	}
//	std::cout << offset << std::endl;
	for (auto row: mData) {
		double *val = (double *)(row + offset);
		out.push_back(*val);
	}
	return out;
}

size_t CSVReader::calculateRowLength() {
	size_t len = 0;;
	for(auto type:mDataTypes) {
		switch(type) {
		case STRING:
			len += maxStringSize * sizeof (char);
			break;
		case INTEGER:
			len += sizeof (int32_t);
			break;
		case REAL:
			len += sizeof (double);
			break;
		case BOOLEAN:
			len += sizeof (bool);
			break;
		case NONE:
			break;
		}
	}
//	std::cout << len << std::endl;
	return len;
}
