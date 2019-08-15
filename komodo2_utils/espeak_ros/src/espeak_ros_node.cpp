/*
 * espeak_ros_node.cpp
 *
 * Originally based on https://github.com/Teknoman117/linbot
 *
 * Modified by Murilo FM (muhrix@gmail.com)
 * 12 Dec 2013
 *
 */

#include <speak_lib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>

#include <dynamic_reconfigure/server.h>
#include <espeak_ros/EspeakConfig.h>

boost::mutex mtx;

std::string getVoiceName(int v);
std::string getDialectName(int d);

void dyn_cfg_callback(espeak_ros::EspeakConfig &cfg, uint32_t level);

void espeak_callback(const std_msgs::String::ConstPtr& line) {
	// lock mutex before calling espeak functions
	boost::mutex::scoped_lock u_lock(mtx);
    /* Speak the string */
	ROS_INFO("[espeak_node]: speaking \"%s\"", line->data.c_str());
    espeak_Synth(line->data.c_str(), line->data.length()+1, 0, POS_CHARACTER, 0, 
        espeakCHARS_AUTO | espeakPHONEMES | espeakENDPAUSE, NULL, NULL);
    espeak_Synchronize();
    //ROS_INFO("Speaking: \"%s\"", line->data.c_str());
}

int main( int argc, char** argv ) {
	espeak_Initialize(AUDIO_OUTPUT_PLAYBACK, 0, NULL, 0);

    ros::init(argc, argv, "espeak_node");
    ros::NodeHandle n;
    ros::NodeHandle priv_n("~");

    // for list of voices, run "espeak --voices"
    // voices: "brazil", "default", "english", "lancashire",
    // "english-rp", "english-wmids", "english-us", "en-scottish"
    std::string voice_name;
    // dialects: "pt-br", "en", "en-uk", "en-uk-north",
    // "en-uk-rp", "en-uk-wmids", "en-us", "en-sc"
    std::string dialect;
    espeak_VOICE voice_select;

    // See speak_lib.h for details and range of the parameters below
    int rate, volume, pitch, range, punctuation;
    int voice_num, dialect_num, capitals, wordgap, age, gender;

    // Fetch default parameters
    rate = espeak_GetParameter(espeakRATE, 0);
    volume = espeak_GetParameter(espeakVOLUME, 0);
    pitch = espeak_GetParameter(espeakPITCH, 0);
    range = espeak_GetParameter(espeakRANGE, 0);
    punctuation = espeak_GetParameter(espeakPUNCTUATION, 0);
    capitals = espeak_GetParameter(espeakCAPITALS, 0);
    wordgap = espeak_GetParameter(espeakWORDGAP, 0);

    voice_num = 1;
    dialect_num = 1;

    // Fetch (and set if param could not be retrieved) ROS parameters
    if (priv_n.getParam("voice", voice_num) == false) {
    	priv_n.setParam("voice", voice_num);
    }
    if (priv_n.getParam("dialect", dialect_num) == false) {
    	priv_n.setParam("dialect", dialect_num);
    }
    if (priv_n.getParam("rate", rate) == false) {
    	priv_n.setParam("rate", rate);
    }
    if (priv_n.getParam("volume", volume) == false) {
    	priv_n.setParam("volume", volume);
    }
    if (priv_n.getParam("pitch", pitch) == false) {
    	priv_n.setParam("pitch", pitch);
    }
    if (priv_n.getParam("punctuation", punctuation) == false) {
    	priv_n.setParam("punctuation", punctuation);
    }
    if (priv_n.getParam("capitals", capitals) == false) {
    	priv_n.setParam("capitals", capitals);
    }
    if (priv_n.getParam("wordgap", wordgap) == false) {
    	priv_n.setParam("wordgap", wordgap);
    }
    priv_n.param("age", age, int(0));
    priv_n.param("gender", gender, int(2));
    if (age < 0 || age > 100) {
    	age = 0;
    }
    if (gender < 0 || gender > 2) {
    	gender = 2;
    }

    voice_name = getVoiceName(voice_num);
    dialect = getDialectName(dialect_num);

    std::memset(&voice_select,0,sizeof(voice_select));
    voice_select.name = voice_name.c_str();
	voice_select.languages = dialect.c_str();
	voice_select.age = age; // 0=not specified, or age in years
	voice_select.gender = gender; // 0=none, 1=male, 2=female

    // set espeak parameters
    //if (espeak_SetVoiceByName(voice_name.c_str()) != EE_OK) {
    //	ROS_ERROR("Could not set espeak voice. Aborting.");
    //	return 1;
    //}
    if (espeak_SetVoiceByProperties(&voice_select) != EE_OK) {
    	ROS_ERROR("Could not set espeak voice properties. Aborting.");
    	return 1;
    }
    //ROS_INFO("Using voice %s", voice_name.c_str());
    //ROS_INFO("Using dialect %s", dialect.c_str());

    if (rate < 80 || rate > 450) {
    	ROS_INFO("Parameter rate = %d out of range, using default value", rate);
    }
    else {
    	if (espeak_SetParameter(espeakRATE, rate, 0) != EE_OK) {
        	ROS_ERROR("Could not set espeak rate. Aborting.");
        	return 1;
    	}
    }
    if (volume < 0 || volume > 200) {
    	ROS_INFO("Parameter volume = %d out of range, using default value", volume);
    }
    else {
    	if (espeak_SetParameter(espeakVOLUME, volume, 0) != EE_OK) {
    		ROS_ERROR("Could not set espeak volume. Aborting.");
    		return 1;
    	}
    }
    if (pitch < 0 || pitch > 100) {
    	ROS_INFO("Parameter pitch = %d out of range, using default value", pitch);
    }
    else {
    	if (espeak_SetParameter(espeakPITCH, pitch, 0) != EE_OK) {
    		ROS_ERROR("Could not set espeak pitch. Aborting.");
    		return 1;
    	}
    }
    if (range < 0 || range > 100) {
    	ROS_INFO("Parameter range = %d out of range, using default value", range);
    }
    else {
    	if (espeak_SetParameter(espeakRANGE, range, 0) != EE_OK) {
    		ROS_ERROR("Could not set espeak range. Aborting.");
    		return 1;
    	}
    }
    if (punctuation < 0 || punctuation > 2) {
    	ROS_INFO("Parameter punctuation out of range, using default value");
    }
    else {
    	if (espeak_SetParameter(espeakPUNCTUATION,
    			espeak_PUNCT_TYPE(punctuation), 0) != EE_OK) {
    		ROS_ERROR("Could not set espeak punctuation. Aborting.");
    		return 1;
    	}
    }
    if (capitals < 0 || capitals > 3) {
    	ROS_INFO("Parameter capitals out of range, using default value");
    }
    else {
    	if (espeak_SetParameter(espeakCAPITALS, capitals, 0) != EE_OK) {
    		ROS_ERROR("Could not set espeak capitals. Aborting.");
    		return 1;
    	}
    }
    if (wordgap < 0 || wordgap > 1000 || wordgap % 10 != 0) {
    	ROS_INFO("Parameter wordgap out of range, using default value");
    }
    else {
    	if (espeak_SetParameter(espeakWORDGAP, wordgap, 0) != EE_OK) {
    		ROS_ERROR("Could not set espeak wordgap. Aborting.");
    		return 1;
    	}
    }

    dynamic_reconfigure::Server<espeak_ros::EspeakConfig> server;
    dynamic_reconfigure::Server<espeak_ros::EspeakConfig>::CallbackType f;
    f = boost::bind(&dyn_cfg_callback, _1, _2);
    server.setCallback(f);

    ros::Subscriber sub = n.subscribe("/espeak_node/speak_line", 20, espeak_callback);

    ros::spin();
    while (n.ok());
    return 0;
}

void dyn_cfg_callback(espeak_ros::EspeakConfig &cfg, uint32_t level) {
	// lock mutex before calling espeak functions
	boost::mutex::scoped_lock u_lock(mtx);

	std::string voice, dialect;
	voice = getVoiceName(cfg.voice);
	dialect = getDialectName(cfg.dialect);

	espeak_VOICE voice_select;
    std::memset(&voice_select,0,sizeof(voice_select));
    voice_select.name = voice.c_str();
	voice_select.languages = dialect.c_str();
	voice_select.age = cfg.age; // 0=not specified, or age in years
	voice_select.gender = cfg.gender; // 0=none, 1=male, 2=female

    if (espeak_SetVoiceByProperties(&voice_select) != EE_OK) {
    	ROS_ERROR("[espeak_node]: Could not set espeak voice properties. Aborting.");
    	return;
    }
    //ROS_INFO("[espeak_node]: Using voice %s", voice.c_str());
    //ROS_INFO("[espeak_node]: Using dialect %s", dialect.c_str());

	if (espeak_SetParameter(espeakRATE, cfg.rate, 0) != EE_OK) {
    	ROS_ERROR("[espeak_node]: Could not set espeak rate. Aborting.");
    	return;
	}
	if (espeak_SetParameter(espeakVOLUME, cfg.volume, 0) != EE_OK) {
		ROS_ERROR("[espeak_node]: Could not set espeak volume. Aborting.");
		return;
	}
	if (espeak_SetParameter(espeakPITCH, cfg.pitch, 0) != EE_OK) {
		ROS_ERROR("[espeak_node]: Could not set espeak pitch. Aborting.");
		return;
	}
	if (espeak_SetParameter(espeakRANGE, cfg.range, 0) != EE_OK) {
		ROS_ERROR("[espeak_node]: Could not set espeak range. Aborting.");
		return;
	}
	if (espeak_SetParameter(espeakPUNCTUATION,
			espeak_PUNCT_TYPE(cfg.punctuation), 0) != EE_OK) {
		ROS_ERROR("[espeak_node]: Could not set espeak punctuation. Aborting.");
		return;
	}
	if (espeak_SetParameter(espeakCAPITALS, cfg.capitals, 0) != EE_OK) {
		ROS_ERROR("[espeak_node]: Could not set espeak capitals. Aborting.");
		return;
	}
	int wordgap = cfg.wordgap % 10;
	if (espeak_SetParameter(espeakWORDGAP, wordgap, 0) != EE_OK) {
		ROS_ERROR("[espeak_node]: Could not set espeak wordgap. Aborting.");
		return;
	}
}

std::string getVoiceName(int v) {
	std::string voice;
	switch (v) {
		case 0: voice.assign("default");
			break;
		case 1: voice.assign("english");
			break;
		case 2: voice.assign("lancashire");
			break;
		case 3: voice.assign("english-rp");
			break;
		case 4: voice.assign("english-wmids");
			break;
		case 5: voice.assign("english-us");
			break;
		case 6: voice.assign("en-scottish");
			break;
		case 7: voice.assign("brazil");
			break;
		default: voice.assign("english");
			break;
	}
	return voice;
}
std::string getDialectName(int d) {
	std::string dialect;
	switch (d) {
		case 0: dialect.assign("en");
			break;
		case 1: dialect.assign("en-uk");
			break;
		case 2: dialect.assign("en-uk-north");
			break;
		case 3: dialect.assign("en-uk-rp");
			break;
		case 4: dialect.assign("en-uk-wmids");
			break;
		case 5: dialect.assign("en-us");
			break;
		case 6: dialect.assign("en-sc");
			break;
		case 7: dialect.assign("pt-br");
			break;
		default: dialect.assign("en-uk");
			break;
	}
	return dialect;
}
