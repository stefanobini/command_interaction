import os
import regex as re

CATEGO = {"power_tool", "hammer"}
EXCLUDES = ["desktop.ini", "db_eng.csv", "db_ita.csv", "info.csv", "sync.ffs_db"]


def get_curr_dir(fil):
    return os.path.abspath(os.path.dirname(os.path.relpath(fil)))

def select_analyzator(service_name):
    def analyze_str_azure(filname):
        voice = None
        if 'mc_' in filname[0:3] or 'take_' in filname[0:5]:
            voice = filname.split("_")[1]
        else:
            voice = filname.split("_")[0]
        
        regex = '[0-9]{1,2}\.(wav|mp3)$'
        cmd_index = int(re.search(pattern=regex, string=filname).group().split('.')[0])
        # cmd_index = int(filname.split("_")[1].split(".")[0])
        return voice, cmd_index

    def analyze_str_google(filname):
        split = filname.split("_")
        voice = None
        if 'mc_' in filname[0:3] or 'take_' in filname[0:5]:
            voice = split[1] + split[2] + split[3]
        else:
            voice = split[0] + split[1] + split[2]
        
        regex = '[0-9]{1,2}\.(wav|mp3)$'
        cmd_index = int(re.search(pattern=regex, string=filname).group().split('.')[0])
        # cmd_index = int(split[3].split(".")[0].split("cmd")[1])
        return voice, cmd_index

    def analyze_str_nemo(filname):
        split = filname.split("_")
        voice = None
        if 'mc_' in filname[0:3] or 'take_' in filname[0:5]:
            voice = split[1]+split[2]
        else:
            voice = split[0]+split[1]
        
        regex = '[0-9]{1,2}\.(wav|mp3)$'
        cmd_index = int(re.search(pattern=regex, string=filname).group().split('.')[0])
        # cmd_index = int(split[2].split(".")[0].split("cmd")[1])
        return voice, cmd_index

    def analyze_str_naturaltts(filname):
        split = filname.split("_")

        voice = None
        if 'mc_' in filname[0:3] or 'take_' in filname[0:5]:
            voice = split[1]
        else:
            voice = split[0]

        regex = '[0-9]{1,2}\.(wav|mp3)$'
        cmd_index = int(re.search(pattern=regex, string=filname).group().split('.')[0])
        return voice, cmd_index

    def analyze_str_real(filname):
        split = filname.split("_")
        voice = None
        if 'mc_' in filname[0:3] or 'take_' in filname[0:5]:
            voice = split[1]
        else:
            voice = split[0]

        regex = '[0-9]{1,2}\.(wav|mp3)$'
        cmd_index = int(re.search(pattern=regex, string=filname).group().split('.')[0])
        return voice, cmd_index
    
    if service_name == "azure":
        return analyze_str_azure
    elif service_name == "google":
        return analyze_str_google
    elif service_name == "ibm":
        return analyze_str_azure
    elif service_name == "nemo":
        return analyze_str_nemo
    elif service_name == "polly":
        return analyze_str_azure
    elif service_name == "vocalware":
        return analyze_str_nemo
    elif service_name == "naturaltts":
        return analyze_str_naturaltts
    elif service_name == "naturalreaders":
        return analyze_str_naturaltts
    elif service_name == "real":
        return analyze_str_real
    else:
        raise Exception("service_name not valid")