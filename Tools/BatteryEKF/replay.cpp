#include "BatteryEKF.h"
#include "DFParser.h"
#include <Eigen/Dense>
#include "json.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <map>
#include <fcntl.h>
#include <sys/mman.h>

using nlohmann::json;
using namespace std;
using namespace Eigen;

template <typename Derived>
static json matrix_to_json(const MatrixBase<Derived>& m);
const char* state_names[] = {
    "SOC","SOH_inv","V1","V2","R0","R1","R2"
};

static const struct {
    const char* name;
    float default_value;
} param_desc_list[] = {
    {"SOH_init",     0.8},
    {"R0_init",      0.048},
    {"R1_init",      3e-3},
    {"R2_init",      3e-3},
    {"RC1",          39},
    {"RC2",          3.6},
    {"I_sigma",      0.05},
    {"SOH_sigma",    0},
    {"R0_sigma",     0.02},
    {"R1_sigma",     3e-3},
    {"R2_sigma",     3e-3},
    {"I_step_sigma", 5},
    {"V_sigma",      0.02},
    {"Q",            12060},
    {"R0_pnoise",    1e-6},
    {"R1_pnoise",    1e-7},
    {"R2_pnoise",    1e-7},
    {"SOC_pnoise",   0}
};

static float soc_ocv_x[] = {0.0, 0.005063014925373088, 0.01613838805970147, 0.02905964179104481, 0.04382680597014932, 0.060439850746268675, 0.07705289552238803, 0.09920364179104468, 0.1268920298507462, 0.15642635820895523, 0.19334423880597018, 0.2357997910447761, 0.2708717910447762, 0.2967142985074628, 0.3244027164179104, 0.34839934328358213, 0.3779336417910447, 0.4037761791044776, 0.4388481492537314, 0.462844776119403, 0.4868414029850746, 0.5182216119402985, 0.5551394925373134, 0.5920573731343284, 0.6289752537313433, 0.6695849253731343, 0.7194240895522388, 0.7581878507462687, 0.7932598507462687, 0.8283318507462687, 0.8615579402985074, 0.9058594029850746, 0.9446231641791045, 0.9815410447761194, 1.0};

static float soc_ocv_y[] = {2.5180000000000002, 2.6487000000000003, 2.75, 2.8668, 2.9681, 3.0693, 3.1550000000000002, 3.2406, 3.3107, 3.373, 3.4198, 3.4587, 3.4899, 3.5132, 3.5288, 3.5444, 3.5678, 3.5911, 3.6145, 3.6456, 3.6612, 3.7001, 3.7313, 3.7702, 3.8014, 3.8403, 3.8793, 3.9104, 3.9494000000000002, 3.9961, 4.027299999999999, 4.0584, 4.074, 4.1051, 4.158};

void process_input_file(string filename, json& data) {
    int fp = open(filename.c_str(), O_RDONLY, 0);
    size_t logdata_len = lseek(fp, 0, SEEK_END);
    lseek(fp, 0, SEEK_SET);
    uint8_t* logdata = (uint8_t*)mmap(0, logdata_len, PROT_READ, MAP_SHARED, fp, 0);
    madvise(logdata, logdata_len, POSIX_MADV_SEQUENTIAL);

    DFParser parser(logdata, logdata_len);
    DFParser::message_t msg;

    data["filename"] = filename;
    data["t"] = json::array();
    data["volt"] = json::array();
    data["curr"] = json::array();
    data["currtot"] = json::array();
    data["voltr"] = json::array();
    data["R"] = json::array();
    while (parser.next_message(msg)) {
        uint8_t instance;
        if(parser.get_message_name(msg) == "BAT") {
            if (parser.get_scalar_field(msg, "Instance", instance) && instance == 0) {
                uint64_t t_us;
                float volt;
                float curr;
                float currtot;
                float voltr;
                float R;

                assert(parser.get_scalar_field(msg, "TimeUS", t_us));
                assert(parser.get_scalar_field(msg, "Volt", volt));
                assert(parser.get_scalar_field(msg, "Curr", curr));
                assert(parser.get_scalar_field(msg, "CurrTot", currtot));
                assert(parser.get_scalar_field(msg, "VoltR", voltr));
                assert(parser.get_scalar_field(msg, "Res", R));

                data["t"].push_back(t_us*1e-6);
                data["volt"].push_back(volt);
                data["curr"].push_back(curr);
                data["currtot"].push_back(currtot);
                data["voltr"].push_back(voltr);
                data["R"].push_back(R);
            }
        }
    }
    munmap(logdata, logdata_len);
    close(fp);
}

int main(int argc, char** argv) {
//     cout << scientific << setprecision(5) << setw(9);
    if (argc != 4) {
        cout << "usage: " << argv[0] << "<config file> <input file> <output file>" << endl;
        return 1;
    }

    ofstream out_file(argv[3]);

    json conf_json;
    { ifstream conf_file(argv[1]); conf_file >> conf_json; }

    union {
        BatteryEKF::Params as_struct;
        float as_array[sizeof(BatteryEKF::Params)/sizeof(float)];
    } params;

    for (uint8_t i=0; i<sizeof(param_desc_list)/sizeof(param_desc_list[0]); i++) {
        auto name = param_desc_list[i].name;
        auto default_value = param_desc_list[i].default_value;
        if (conf_json[name].is_number()) {
            params.as_array[i] = (float)conf_json[name];
            cout << "param " << name << " " << params.as_array[i] << endl;
        } else {
            params.as_array[i] = default_value;
            cout << "param " << name << " " << params.as_array[i] << " DEFAULT" << endl;
        }
    }

    // INITIALIZE EKF

    json data;
    try {
        ifstream in_file(argv[2]);
        char buf[1024*16];
        in_file.rdbuf()->pubsetbuf(buf, sizeof(buf));
        data = json::parse(in_file);
        in_file.close();
    } catch(...) {
        process_input_file(argv[2],data);
    }

    float prev_t = float(data["t"][0]);
    float V = float(data["volt"][0])/6;
    float I = float(data["curr"][0]);

    json output = data;
    output["conf"] = conf_json;
    output["state_names"] = json::array();
    for (uint8_t i=0; i<sizeof(state_names)/sizeof(state_names[0]); i++) {
        output["state_names"].push_back(state_names[i]);
    }

    output["dt"] = json::array();
    output["y"] = json::array();
    output["NIS"] = json::array();
    output["E"] = json::array();
    output["stddev"]["E"] = json::array();

    auto& dt_series = output["dt"];
    auto& y_series = output["y"];
    auto& NIS_series = output["NIS"];
    dt_series.push_back(0);
    y_series.push_back(0);
    NIS_series.push_back(0);

    BatteryChemistryModelLinearInterpolated chemistry_model(soc_ocv_x, soc_ocv_y, sizeof(soc_ocv_x)/sizeof(*soc_ocv_x));
    BatteryEKF ekf(params.as_struct, chemistry_model);

    ekf.initialize(V,I,25);

    output["E"].push_back(ekf.get_remaining_energy_J(25));
    output["stddev"]["E"].push_back(ekf.get_remaining_energy_J_sigma(25));


    for (auto& name : state_names) {
        output[name] = json::array();
        output["stddev"][name] =  json::array();

        output[name].push_back(ekf.get_state()(&name-&state_names[0]));
        output["stddev"][name].push_back(sqrt(ekf.get_covariance()(&name-&state_names[0], &name-&state_names[0])));
    }

    float reinit_timer = 0;

    for (size_t i=1; i<data["t"].size(); i++) {
        float t = float(data["t"][i]);
        float dt = t-prev_t;

        V = float(data["volt"][i])/6;
        I = float(data["curr"][i]);

        reinit_timer += dt;
        if (reinit_timer > 3600) {
//             ekf.initialize(V,I);
            reinit_timer = 0;
        }

        ekf.predict(dt,I);

//         cout << ekf.get_state().transpose() << endl << endl;

        for (auto& name : state_names) {
            output[name].push_back(ekf.get_state()(&name-&state_names[0]));
            output["stddev"][name].push_back(sqrt(ekf.get_covariance()(&name-&state_names[0], &name-&state_names[0])));
        }

        float y=0, NIS=0;
        ekf.update(V,I,25,y,NIS);
        dt_series.push_back(dt);
        y_series.push_back(y);
        NIS_series.push_back(NIS);
        output["E"].push_back(ekf.get_remaining_energy_J(25));
        output["stddev"]["E"].push_back(ekf.get_remaining_energy_J_sigma(25));

        prev_t = t;
//         if (i > 100) break;
    }

    out_file << output;

    out_file.close();

    return 0;
}


template <typename Derived>
static json matrix_to_json(const MatrixBase<Derived>& m) {
    json ret = json::array();

    for (int i=0; i<m.rows(); i++) {
        json rowdata = json::array();
        for (int j=0; j<m.cols(); j++) {
            rowdata.push_back(m(i,j));
        }

        ret.push_back(rowdata);
    }

    return ret;
}
