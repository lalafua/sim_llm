import rospy

class HighlightLog:
    @staticmethod   
    def log_success(msg):
        rospy.loginfo(f"\033[92m[SUCCESS] {msg}\033[0m")
    
    @staticmethod 
    def log_highlight(msg):
        rospy.loginfo(f"\033[96m[INFO] {msg}\033[0m")
    
    @staticmethod    
    def log_alert(msg):
        rospy.logwarn(f"\033[93m[ALERT] {msg}\033[0m")

    @staticmethod 
    def log_error(msg):
        rospy.logerr(f"\033[91m[ERROR] {msg}\033[0m")
