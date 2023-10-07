/*
* 语音听写(iFly Auto Transform)技术能够实时地将语音转换成对应的文字。
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"
#include <iconv.h>
#include <regex.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096

#define MAX_PARAMS_LEN      (1024)
#define MAX_GRAMMARID_LEN   (32)
#define SAMPLE_RATE_16K     (16000)
#define SAMPLE_RATE_8K      (8000)

const char * GRM_FILE            = "/home/zq/catkin_ws/src/xfei_asr/bin/call.bnf"; //构建离线识别语法网络所用的语法文件
const char * ASR_RES_PATH        = "fo|/home/zq/catkin_ws/src/xfei_asr/bin/msc/res/asr/common.jet"; //离线语法识别资源路径
const char * GRM_BUILD_PATH      = "/home/zq/catkin_ws/src/xfei_asr/bin/msc/res/asr/GrmBuilld"; //构建离线语法识别网络生成数据保存路径

char* rawtextValue;

int flag    = 0 ;
int flag_ok = 0 ;
int flag_no = 0 ;
int flag_none = 1 ;
typedef struct _UserData {
	int     build_fini; //标识语法构建是否完成
	int     update_fini; //标识更新词典是否完成
	int     errcode; //记录语法构建或更新词典回调错误码
	char    grammar_id[MAX_GRAMMARID_LEN]; //保存语法构建返回的语法ID
}UserData;

int build_grammar(UserData *udata); //构建离线识别语法网络
int run_asr(UserData *udata); //进行离线语法识别

int build_grm_cb(int ecode, const char *info, void *udata)
{
	UserData *grm_data = (UserData *)udata;

	if (NULL != grm_data) {
		grm_data->build_fini = 1;
		grm_data->errcode = ecode;
	}

	if (MSP_SUCCESS == ecode && NULL != info) {
		printf("构建语法成功！ 语法ID:%s\n", info);
		if (NULL != grm_data)
			snprintf(grm_data->grammar_id, MAX_GRAMMARID_LEN - 1, info);
	}
	else
		printf("构建语法失败！%d\n", ecode);

	return 0;
}

int build_grammar(UserData *udata)
{
	FILE *grm_file                           = NULL;
	char *grm_content                        = NULL;
	unsigned int grm_cnt_len                 = 0;
	char grm_build_params[MAX_PARAMS_LEN]    = {NULL};
	int ret                                  = 0;

	grm_file = fopen(GRM_FILE, "rb");
	if(NULL == grm_file) {
		printf("打开\"%s\"文件失败！[%s]\n", GRM_FILE, strerror(errno));
		return -1;
	}

	fseek(grm_file, 0, SEEK_END);
	grm_cnt_len = ftell(grm_file);
	fseek(grm_file, 0, SEEK_SET);

	grm_content = (char *)malloc(grm_cnt_len + 1);
	if (NULL == grm_content)
	{
		printf("内存分配失败!\n");
		fclose(grm_file);
		grm_file = NULL;
		return -1;
	}
	fread((void*)grm_content, 1, grm_cnt_len, grm_file);
	grm_content[grm_cnt_len] = '\0';
	fclose(grm_file);
	grm_file = NULL;

    snprintf(grm_build_params, MAX_PARAMS_LEN - 1,
		"engine_type = local, \
		asr_res_path = %s, sample_rate = %d, \
		grm_build_path = %s, ",
		ASR_RES_PATH,
		SAMPLE_RATE_16K,
		GRM_BUILD_PATH
		);

	ret = QISRBuildGrammar("bnf", grm_content, grm_cnt_len, grm_build_params, build_grm_cb, udata);

	free(grm_content);
	grm_content = NULL;

	return ret;
}

char* extractRawTextFromXML(const char* xml_data) {
    regex_t preg;
    regmatch_t matches[2];
    const char* pattern = "<rawtext>(.*?)</rawtext>";

    if (regcomp(&preg, pattern, REG_EXTENDED) != 0) {
        printf("Failed to compile regex pattern.\n");
        return NULL;
    }

    if (regexec(&preg, xml_data, 2, matches, 0) == 0) {
        int start = matches[1].rm_so;
        int end = matches[1].rm_eo;
        int length = end - start;
        rawtextValue = (char *)malloc(length + 1);
        memcpy(rawtextValue, xml_data + start, length);
        rawtextValue[length] = '\0';
        regfree(&preg);
        return rawtextValue;
    }

    regfree(&preg);
    return NULL;
}

static void show_result(char *string, char is_over)
{
    flag_ok=1;	
//    printf("\rResult:[ %s ]",  string);
    const char* xml_data = string;
    rawtextValue = extractRawTextFromXML(xml_data);
    if (rawtextValue != NULL) {
        printf("\nRaw Text: %s\n", rawtextValue);
		flag_no = 0;
    }
	else flag_no=1;
		
    if(is_over)
		putchar('\n');

}

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

void on_result(const char *result, char is_last)
{
	if (result) {
		size_t left = g_buffersize - 1 - strlen(g_result);
		size_t size = strlen(result);
		if (left < size) {
			g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
			if (g_result)
				g_buffersize += BUFFER_SIZE;
			else {
				printf("mem alloc failed\n");
				return;
			}
		}
		strncat(g_result, result, size);
		show_result(g_result, is_last);
		flag_none = 0;
	}
}
void on_speech_begin()
{
	if (g_result)
	{
		free(g_result);
	}
	g_result = (char*)malloc(BUFFER_SIZE);
	g_buffersize = BUFFER_SIZE;
	memset(g_result, 0, g_buffersize);

	printf("Start Listening...\n");
}
void on_speech_end(int reason)
{
	if (reason == END_REASON_VAD_DETECT)
		printf("\nSpeaking done \n");
	else
		printf("\nRecognizer error %d\n", reason);
}


/* demo recognize the audio from microphone */
static void demo_mic(const char* session_begin_params)
{
	int errcode;
	int i = 0;

	struct speech_rec iat;

	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
	if (errcode) {
        flag_no=1;
		printf("speech recognizer init failed\n");
		return;
	}
	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("start listen failed %d\n", errcode);
	}
	/* demo 15 seconds recording */
	while(i++ < 5)
		sleep(1);

	if(flag_none){
			flag_no = 1;
	}	
	errcode = sr_stop_listening(&iat);
	if (errcode) {
        flag_no=1;
		printf("stop listening failed %d\n", errcode);
	}
	
	sr_uninit(&iat);
}


/* main thread: start/stop record ; query the result of recgonization.
 * record thread: record callback(data write)
 * helper thread: ui(keystroke detection)
 */

void WakeUp(const std_msgs::String::ConstPtr& msg)
{
    printf("waking up\r\n");
   // printf("%s", *msg->data.c_str());
    usleep(700*1000);
    flag=1;
}

int run_asr(UserData *udata)
{
	char asr_params[MAX_PARAMS_LEN]    = {NULL};

	//离线语法识别参数设置
	snprintf(asr_params, MAX_PARAMS_LEN - 1,
		"engine_type = local, \
		asr_res_path = %s, sample_rate = %d, \
		grm_build_path = %s, local_grammar = %s, \
		result_type = xml, result_encoding = UTF-8, ",
		ASR_RES_PATH,
		SAMPLE_RATE_16K,
		GRM_BUILD_PATH,
		udata->grammar_id
		);

		demo_mic(asr_params);

	return 0;
}


int main(int argc, char* argv[])
{

    ros::init(argc, argv, "xfspeech");
    ros::NodeHandle n;
    // ros::Publisher pub = n.advertise<std_msgs::String>("xfspeech", 1000);
    ros::Rate loop_rate(10);

    ros::Subscriber sbu = n.subscribe("xfwakeup", 1000, WakeUp);
    // ros::Publisher pub1 = n.advertise<std_msgs::String>("xfwords", 1000);
    ros::Publisher pub2 = n.advertise<std_msgs::String>("xfspeech", 1000);

	

    int count=0;

    while(ros::ok())
    {

		flag_none = 1;
        if (flag){
			
			int ret = MSP_SUCCESS;
			const char* login_params = "appid = 20865220, work_dir = .";
			UserData asr_data;

			ret = MSPLogin(NULL, NULL, login_params);
			if(MSP_SUCCESS != ret){
				MSPLogout();
				printf("MSPLogin failed , Error code %d.\n",ret);
			}

			memset(&asr_data, 0, sizeof(UserData));
			printf("构建离线识别语法网络...\n");
			ret = build_grammar(&asr_data);  //第一次使用某语法进行识别，需要先构建语法网络，获取语法ID，之后使用此语法进行识别，无需再次构建

			while (1 != asr_data.build_fini)
				usleep(300 * 1000);

			printf("离线识别语法网络构建完成，开始识别...\n");

            run_asr(&asr_data);
            flag=0;
			MSPLogout();
        }

        if(flag_ok){
            flag_ok=0;
            std_msgs::String msg;
            msg.data = rawtextValue;
            pub2.publish(msg);
        }

        if(flag_no){
            flag_no=0;
            std_msgs::String msg;
            msg.data = "Sorry Please do it again";
            pub2.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
        count++;
        printf("c:%d \r\n", count);
	}

	return 0;
}
