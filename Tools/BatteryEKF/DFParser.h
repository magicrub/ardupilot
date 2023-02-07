#pragma once

#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp> // Include for boost::split
#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>
#include <vector>
#include <map>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "BlockingCollection.h"
#pragma GCC diagnostic pop
#include <thread>

using namespace std;
using namespace code_machina;

#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149

class DFParser {
public:
    typedef struct {
        uint8_t type;
        uint8_t* body;
    } message_t;

    typedef struct {
        typedef struct {
            char typechar;
            string name;
            int ofs;
            int len;
        } Field;

        uint8_t type;
        string name;
        int len;
        vector<Field> fields;
    } Format;


    DFParser(const DFParser&) = delete;
    DFParser& operator=(const DFParser&) = delete;
    DFParser(uint8_t* logdata, size_t logdata_len) : _logdata(logdata), _logdata_len(logdata_len) {
        _formats[0x80] = {
            0x80,
            "FMT",
            89,
            {
                {'B',"Type",0,1},
                {'B',"Length",1,1},
                {'n',"Name",2,4},
                {'N',"Format",6,16},
                {'Z',"Columns",22,64}
            }
        };

        // start parsing thread
        thread parse_thread(&DFParser::parse_thread_func, this);
        parse_thread.detach();
    }

    string get_message_name(const message_t& msg) {
        return _formats[msg.type].name;
    }

    Format::Field* get_field_definition(const message_t& msg, const string& fieldname) {
        auto& fields = _formats[msg.type].fields;
        for (auto& f : fields) {
            if (f.name == fieldname) {
                return &f;
            }
        }
        return NULL;
    }

    template <typename T>
    bool get_scalar_field(const message_t& msg, const string& fieldname, T& ret) {
        Format::Field* field_ptr = get_field_definition(msg, fieldname);
        if (!field_ptr) return false;
        Format::Field& field = *field_ptr;

        switch(field.typechar) {
            case 'B':
            case 'M':
                ret = (T)*reinterpret_cast<const uint8_t*>(&msg.body[field.ofs]);
                return true;
            case 'b':
                ret = (T)*reinterpret_cast<const int8_t*>(&msg.body[field.ofs]);
                return true;
            case 'h':
                ret = (T)*reinterpret_cast<const int16_t*>(&msg.body[field.ofs]);
                return true;
            case 'H':
                ret = (T)*reinterpret_cast<const uint16_t*>(&msg.body[field.ofs]);
                return true;
            case 'i':
                ret = (T)*reinterpret_cast<const int32_t*>(&msg.body[field.ofs]);
                return true;
            case 'I':
                ret = (T)*reinterpret_cast<const uint32_t*>(&msg.body[field.ofs]);
                return true;
            case 'q':
                ret = (T)*reinterpret_cast<const int64_t*>(&msg.body[field.ofs]);
                return true;
            case 'Q':
                ret = (T)*reinterpret_cast<const uint64_t*>(&msg.body[field.ofs]);
                return true;
            case 'f':
                ret = (T)*reinterpret_cast<const float*>(&msg.body[field.ofs]);
                return true;
            case 'L':
                ret = (T)*reinterpret_cast<const uint32_t*>(&msg.body[field.ofs]);
                ret /= 1e7;
                return true;
            case 'd':
                ret = (T)*reinterpret_cast<const double*>(&msg.body[field.ofs]);
                return true;
            case 'c':
                ret = (T)*reinterpret_cast<const int16_t*>(&msg.body[field.ofs]);
                ret /= 100;
                return true;
            case 'C':
                ret = (T)*reinterpret_cast<const uint16_t*>(&msg.body[field.ofs]);
                ret /= 100;
                return true;
            case 'e':
                ret = (T)*reinterpret_cast<const int32_t*>(&msg.body[field.ofs]);
                ret /= 100;
                return true;
            case 'E':
                ret = (T)*reinterpret_cast<const uint32_t*>(&msg.body[field.ofs]);
                ret /= 100;
                return true;
        }
        return false;
    }
    template<typename T> bool get_scalar_field(const message_t& msg, const string& fieldname, uint8_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const string& fieldname, int8_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const string& fieldname, uint16_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const string& fieldname, int16_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const string& fieldname, uint32_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const string& fieldname, int32_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const string& fieldname, int64_t& ret);
    template<typename T> bool get_scalar_field(const message_t& msg, const string& fieldname, uint64_t& ret);

    bool get_string_field(const message_t& msg, const string& fieldname, string& ret) {
        Format::Field* field_ptr = get_field_definition(msg, fieldname);
        if (!field_ptr) return false;
        Format::Field& field = *field_ptr;

        switch(field.typechar) {
            case 'n':
            case 'N':
            case 'Z': {
                ret = "";
                for (int i=field.ofs; i<field.ofs+field.len; i++) {
                    if (msg.body[i] == 0) break;
                    ret.push_back((char)msg.body[i]);
                }
                return true;
            }
        }
        return false;
    }

    void process_fmt(const message_t& msg) {
        Format new_format;

        get_scalar_field(msg, "Type", new_format.type);
        get_scalar_field(msg, "Length", new_format.len);

        get_string_field(msg, "Name", new_format.name);
        string fmtstr;
        get_string_field(msg, "Format", fmtstr);
        vector<string> fieldnames;

        string colstr;
        get_string_field(msg, "Columns", colstr);
        boost::split(fieldnames, colstr, boost::is_any_of(","));

        assert(fieldnames.size() == fmtstr.length());

        int ofs = 0;
        for (size_t i=0; i<fieldnames.size(); i++) {
            int size =  _fieldsizemap[fmtstr[i]];
            new_format.fields.push_back({fmtstr[i], fieldnames[i], ofs, size});
            ofs += size;
        }
//         cout << new_format.name << " " << fmtstr << " " << ofs << " " << new_format.len-3 << endl;
        assert(ofs == new_format.len-3);
        _formats[int(new_format.type)] = new_format;
    }

    bool next_message(message_t& msg) {
        auto status = _msgqueue.take(msg);
        return status == BlockingCollectionStatus::Ok;
    }

private:

    void parse_thread_func() {
        message_t msg;
        while (parse_next(msg)) {

            if (msg.type == 0x80) {
                process_fmt(msg);
            }

            _msgqueue.add(msg);
        }

        _msgqueue.complete_adding();
    }

    bool parse_next(message_t& msg) {
        if (_logdata_len-_ofs < 3) {
            return false;
        }
        uint8_t* header = &_logdata[_ofs];

        int skip_count = 0;
        while (header[0] != HEAD_BYTE1 || header[1] != HEAD_BYTE2 || (_formats.find(header[2]) == _formats.end())) {
            if (_logdata_len-_ofs < 3) {
                return false;
            }
            skip_count++;
            _ofs++;
            header = &_logdata[_ofs];
        }

        if (skip_count > 0) {
            cerr << "skipped " << skip_count << " bytes" << endl;
        }

        _ofs += 3;

        msg.type = header[2];

        size_t body_len = _formats[msg.type].len-3;

        if(_logdata_len-_ofs < body_len) {
            return false;
        }

        msg.body = &_logdata[_ofs];

        _ofs += body_len;

        return true;
    }

    uint8_t* _logdata;
    size_t _logdata_len;
    size_t _ofs {};
    map<uint8_t,Format> _formats;
    BlockingCollection<message_t> _msgqueue;

    map<char,uint8_t> _fieldsizemap = {
        {'b', 1},
        {'B', 1},
        {'M', 1},
        {'h', 2},
        {'H', 2},
        {'i', 4},
        {'I', 4},
        {'q', 8},
        {'Q', 8},
        {'n', 4},
        {'N', 16},
        {'Z', 64},
        {'c', 2},
        {'C', 2},
        {'e', 4},
        {'E', 4},
        {'f', 4},
        {'d', 8},
        {'L', 4},
        {'a', 64}
    };
};
