import pymysql
import datetime


class MySQL():
    def connect(self):
        '''连接MySQL数据库'''
        try:
            self.db = pymysql.connect(
                host='192.168.139.18',
                user='camera1',
                passwd='Erleng921010',
                port=3306,
                db='robot_positions',
                charset='utf8'
            )
            self.cursor = self.db.cursor()
            print("success")
            return self.db
        except:
            print("failed")
            raise Exception("数据库连接失败")

    def insert(self, robot_pos = ''):
        '''
        执行SQL语句
        robot_pos 为 0~14 机器人坐标, 位置：3000mm*5000mm, 角度：0-360度, 逗号开头
        e.g. ,'354','1665','28.1'...
        '''
        sql = "INSERT INTO robot_pos VALUES ("
        # 插入毫秒级时间
        localtime_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        sql += "str_to_date('%s', '%%Y-%%m-%%d%%H:%%i:%%S.%%f')" % (localtime_str)
        if robot_pos =='':
            for j in range(15):
                sql += ",'354','1665','28.1'"
        else:
            sql += robot_pos
        sql += ")"
        # try:
        self.cursor.execute(sql)
        # result = self.cursor.fetchone()
        self.db.commit()
        # print('结果', result)
        # except:
        #     db.rollback()
        #     print("失败")

    def check(self):
        '''查看所有数据'''
        sql = "select * from robot_pos"

        try:
            self.cursor.execute(sql)
            result = self.cursor.fetchall()
            self.db.commit()
            print('结果', result)
        except:
            self.db.rollback()
            print("失败")

    def deleteDB(self):
        sql = "DELETE FROM robot_pos"
        self.cursor.execute(sql)
        self.db.commit()

    def close(self):
        self.cursor.close()
        self.db.close()
        
        pass


if __name__ == '__main__':
    robot_positions_DB = MySQL()
    robot_positions_DB.connect()
    # robot_positions_DB.insert()
    # robot_positions_DB.check()
    robot_positions_DB.deleteDB()
    robot_positions_DB.close()
