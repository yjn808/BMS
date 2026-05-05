#include  "my_usart.h"
/*
       ////---------注意---------////
函数使用字符传入注意：小数点后面第一位最好是偶数，不要奇数，硬件计算原因
                     例：456.234  解析出来为456.234
										     456.123  解析出来可能为 456.129999
			////---------注意---------////
*/


/*
函数功能：处理串口接收到的数据--将字符串数据转为数字---整形数据
作者：何荣涛
参数：date:串口接收到的数据
      signed_number;数据头部标号
      BUFF_RX_MAX_DATE：串口的接收数据个数最大数量（也可以是确定最大接受的数据个数）,用于清除已经处理过的数据
      text_buf:用于接受剔除头部标号的数据，将数据保存为字符串便于OLED显示
 
 
函数返回值：正确：解析后的数据  错误： 0xffff数据解析错误
*/
int my_usart_date_int(uint8_t *date,uint16_t signed_number,uint16_t BUFF_RX_MAX_DATE,uint8_t *text_buf)
{
	
	uint16_t number = 0;
	uint16_t head_number = 0;
	int calculate_date = 0;
  //计算字符串个数
	number = my_strlen(date);
	//处理数据标号
	head_number =  head_number_date(date);
	//判断接收数据是否为标号数据
	if(head_number == signed_number)
	{	   
	 calculate_date= Date_number_int(head_number ,number,date,text_buf); 
	}
	
	//判断数据是否错位
	if(number == 0xffff || head_number ==0xffff ||calculate_date ==0xfffff)
	{
	   // printf("数据错误\r\n");
		  date_clear(date, BUFF_RX_MAX_DATE);
		  return 0;
	}
	else
	{
			date_clear(date,BUFF_RX_MAX_DATE);
	}
	return calculate_date;
}

/*
函数功能：处理串口接收到的数据--将字符串数据转为数字---浮点数据
作者：何荣涛
参数：date:串口接收到的数据
      signed_number;数据头部标号
      BUFF_RX_MAX_DATE：串口的接收数据个数最大数量（也可以是确定最大接受的数据个数）,用于清除已经处理过的数据
      text_buf:用于接受剔除头部标号的数据，将数据保存为字符串便于OLED显示
 
       ////---------注意---------////
函数使用字符传入注意：小数点后面第一位最好是偶数，不要奇数，硬件计算原因
			////---------注意---------////

函数返回值：正确：解析后的数据  错误： 0xffff数据解析错误
*/
float my_usart_date_float(uint8_t  *date,uint16_t signed_number,uint16_t BUFF_RX_MAX_DATE,uint8_t *text_buf)
{
	
	uint16_t number = 0;
	uint16_t head_number = 0;
	float calculate_date = 0;
  //计算字符串个数
	number = my_strlen(date);
	//处理数据标号
	head_number =  head_number_date(date);
	//判断接收数据是否为标号数据
	if(head_number == signed_number)
	{	   
	 calculate_date= Date_number_float(head_number ,number,date,text_buf); 
	}
	
	//判断数据是否错位
	if(number == 0xffff || head_number ==0xffff ||calculate_date ==0xfffff)
	{
	    //printf("数据错误\r\n");
		  date_clear(date,BUFF_RX_MAX_DATE);
		  return 0;
	}
	else
	{
			date_clear(date,BUFF_RX_MAX_DATE);
	}
	return calculate_date;
}


/*
函数功能：统计字符串长度
作者：何荣涛
参数：字符串
返回值说明：返回值为从1开始计数，如果需要对应数组下标需要减1
*/
uint16_t  my_strlen(uint8_t  *date)
{
	   uint16_t number = 0;
       while(*date != '\0')
			 {
					date++;	
				  number +=1;
				 
			 }
        return number;
}
/*
函数功能：清除接收到的数据
作者：何荣涛
参数：date：清除数据的地址   number：需要清除的数据个数
*/
void date_clear(uint8_t  *date,uint16_t number)
{
    uint16_t number1 = 0;
	 //如果需要被清除的数据地址为0.则表示无效地址
	 if(date == 0) 
	 {
	    //无效地址，不做处理
	 }
	 else
	 {		 
			for(number1 = 0;number1<number;number1++)
		 {
					date[number1] = 0;
		 }
   }
}
/*
函数功能：获取字符串的头部标号---规定数据标号在1--999范围内
作者：何荣涛
参数：字符串
*/
uint16_t head_number_date(uint8_t  *date)
{
	  int sig_1 = 0;
	  uint8_t sig = 0;
	  uint8_t i = 0;
	  uint16_t head_date = 0;
	  uint16_t head_date1 = 0;
    //首先判断数据标号是否正确
	  sig_1 = text_number(&date[0]);
	  if(sig_1 == 0 || sig_1 ==0xffff) return 0xffff;
	
	  //判断标号数据大小，规定范围在1--999范围内，如需要增加标号，则修改下面代码
	   sig = 0;
	   while(date[sig]!=':' )
		 {
		     sig+=1;
			   if(sig==4) break;
		 }
		 if(sig == 4) return 0xffff;//超出标号范围
	  //处理标号数据
		head_date1 = 0;
		for(i = 0;i<sig;i++)
		{
			head_date1 *=10;
			switch(date[i])
			{
				case '0': head_date = 0;break;
				case '1': head_date = 1;break;
				case '2': head_date = 2;break;
				case '3': head_date = 3;break;
				case '4': head_date = 4;break;
				case '5': head_date = 5;break;
				case '6': head_date = 6;break;
				case '7': head_date = 7;break;
				case '8': head_date = 8;break;
				case '9': head_date = 9;break;
				default : head_date = 0xffff;
			}
			if(head_date == 0xffff)return 0xffff;
			head_date1 +=head_date;
		}

    return head_date1;
}
/*
函数功能：平方运算
作者：何荣涛
参数：date---需要计算的数据  sqr 几次平方
*/
int my_squre(int date,uint16_t sqr)
{
    int date_rei = 0;
    uint8_t i =0;
    if(date == 0) return 0;
    if(date == 1) return 1;
    if(sqr ==0)   return 1;
    if(sqr==1)    return  date;
    if(sqr>=2)
    {
        date_rei = 1;
        for(i=0;i<sqr;i++)
        {
          date_rei = date_rei*date;
        }
    }
     return date_rei;
}
/*
函数功能：单个字符串转换为数字
作者：何荣涛
参数：字符输入
*/
int text_number(uint8_t  * text)
{
     int date1 = 0;

       switch(*text)
       {
           case '0':date1=0;break;
           case '1':date1=1;break;
           case '2':date1=2;break;
           case '3':date1=3;break;
           case '4':date1=4;break;
           case '5':date1=5;break;
           case '6':date1=6;break;
           case '7':date1=7;break;
           case '8':date1=8;break;
           case '9':date1=9;break;
				   default : date1 =0xffff;
       }
   return  date1;
}

/*
函数功能：int整型数据解析处理--
          --将接收到的串口字符串数据转为int整型数据-
          --将接收到的串口字符串数据保留为字符串，方便用于OLED屏幕显示-

作者：何荣涛
参数：head_number :数据头部标号  number字符串总个数  *date需要处理的字符串数据
参数说明1：head_number用于计算头部字符个数
参数说明2：number接受的字符串总个数
参数说明3：*date，串口接收的字符串，字符串保存的位置
参数说明4：*text_buf，接受剔除头部标号的数据，保存为字符串类型，便于OLED显示


*/
int Date_number_int(uint16_t head_number ,uint16_t number,uint8_t *date,uint8_t *text_buf)
{
	
		int receive_date = 0;//返回处理后的数据
		int date1 = 0;
		uint16_t head = 0;    //头部字符串个数_: 两个字符
	  uint8_t  text_head = 0;//字符个数
		uint16_t date_number = 0; //有效数据位个数
		uint8_t signed_1 = 0;       //数据标记1
		uint16_t i = 0;
	  uint8_t text_buff[30];
	
	//判断是否含有小数点----整形数据不能带有小数点
	for(i = 0;i<number;i++)
	{
	   if(date[i] == '.') return 0xfffff;
	
	}
	//计算head位数----head=数据个数 + ：
	 if(head_number >=1 && head_number <=9)
	 {head = 2;}
	 else if (head_number >=10 && head_number <=99)
	 {head = 3;}
	 else if (head_number >=100 && head_number <=999)
	 {head = 4;}
	 else 
	 {return 0xfffff;}
	 
	//计算有效位数据个数
	date_number = number - head;//得到有效数据位个数

	//判断符号位
	if(date[head] == '-' || date[head] == '+') 
	{
		signed_1 = 1;
		text_buff[0] = date[head];//将符号位字符拿出来
		text_head = 1;
	}
	 //如果首位为符号位，则需要重新计算一下有效数据个数，把符号字符减去
	if(signed_1 == 1)
   {
		date_number -=1;
		head +=1;
	 }
	 
	//判断首位数据是否为无效数据--整型数据首位数据不能为0，例如10：-01234，则为无效数据

	  receive_date = text_number(&date[head]);
		if(receive_date == 0 || receive_date == 0xffff) 
		{ 
			date_clear(text_buff,3);//清除前面的字符在退出
			return 0xfffff;
		}

	//开始数据处理---把之前使用过的变量清0，避免造成下面数据解析错误
	  receive_date = 0;
	  date1 = 0;
	for(i = head;i<number;i++)
	 {		
		 text_buff[text_head] = date[i];//将符号位字符拿出来
		 text_head +=1;
	   date1 = text_number(&date[i])*my_squre(10,(date_number-(i-head) - 1));
		 receive_date +=date1;
	 } 
	//如果有负号则加上-
	 if(signed_1 == 1) receive_date *=-1;
	 //将字符串传出去
	 for(i = 0;i<text_head;i++)
	 {
	    text_buf[i] = text_buff[i];
	 }
	 return receive_date;
}

/*
函数功能：float单精度浮点数据解析处理--
          --将接收到的串口字符串数据转为浮点数据-
          --将接收到的串口字符串数据保留为字符串，方便用于OLED屏幕显示-


作者：何荣涛

参数：   head_number :数据头部标号  number字符串总个数  *date需要处理的字符串数据
参数说明1：head_number用于计算头部字符个数
参数说明2：number接收的字符串总个数
参数说明3：*date，串口接收的字符串，字符串保存的位置
参数说明4：*text_buf，接受剔除头部标号的数据，保存为字符串类型，便于OLED显示


注意；STM32F407的float保留小数点后六位，且会有偏差
例  45.456     可能会偏差正负 0.000001
    -80.45679  可能会偏差正负 0.000001

*/
float Date_number_float(uint16_t head_number ,uint16_t number,uint8_t *date,uint8_t *text_buf)
{
		float receive_date = 0;//返回处理后的数据
		float date1 = 0;
	  int   date2 = 0;
	  int   date3 = 0;
		uint16_t head = 0;    //头部字符串个数_: 两个字符
	  uint16_t head2 = 0;
	  uint8_t  text_head = 0;//保存字符个数
		uint16_t date_number = 0; //有效数据位个数
		uint8_t signed_1 = 0;       //数据标记1
	  uint8_t signed_2 = 0;		
	  uint8_t text_buff[30];    //这里限定最大接收字符个数为30个
	  uint16_t i = 0;
	
	//判断是否含有小数点---浮点数据必须带有小数点
	for(i = 0;i<number;i++)
	{
	   if(date[i] == '.') signed_1 = 1;
	}
	if(signed_1 != 1) return 0xfffff;
	signed_1 = 0;
	//计算head位数----head=数据个数 + ：
	 if(head_number >=1 && head_number <=9)
	 {head = 2;}
	 else if (head_number >=10 && head_number <=99)
	 {head = 3;}
	 else if (head_number >=100 && head_number <1000)
	 {head = 4;}
	 else 
	 {return 0xfffff;}
	//计算有效位数据个数
	date_number = number - head;//得到有效数据位个数
	//判断符号位
	if(date[head] == '-' || date[head] == '+')
	{		
		signed_1 = 1;
		text_buff[0] = date[head];//将符号位字符拿出来
		text_head = 1;
	}
	 //如果首位为符号位，则需要重新计算一下有效数据个数，把符号字符减去
	if(signed_1 == 1)
   {
		date_number -=1;
		head +=1;
	 }
	 
	//判断首位是否为0,如果首位为0，则整数部分为0，且下一位必须为小数点
	  date2 = text_number(&date[head]);
	 if(date2 == 0) 
	 {
		 signed_2 = 1;
	 }
	//如果首位为0
	 if(signed_2 == 1)
	 {
  
				//首先判断数据格式是否正确
	      if(date[head+1] !='.')
				{
					date_clear(text_buff,3);
					return 0xfffff;
				}
		    //--------数据格式正确-------
		 		  //将字符保存
				 text_buff[text_head] = '0';
				 text_head+=1;
				 text_buff[text_head] = '.';
				 text_head+=1;
		 
		     head +=2;//跳到小数点后面第一位数
		     date_number -=2;//除掉首位0和小数点
		    //小数点后面数据处理
		     date1 = 0;
		    for(i = head;i<number;i++)
		     {
					 //保存字符
					  text_buff[text_head] = date[i];
					  text_head+=1;
					 //将小数逐个取出，按对应位进行 10*加权
					 //例如取出0.405
					 //4*10（2次方）  0*10（1次方） 5*10（0次方）
					 date2 = text_number(&date[i])*my_squre(10,(date_number-(i-head) - 1));
					 date3 +=date2;
				 }
				//小数点后面数据转换成整数完毕
				date1 = date3;
				//除以放大的倍数，转换会原来的小数
      	receive_date = date1/my_squre(10,date_number);
				if(signed_1 == 1) receive_date*=-1;
					 //将字符串传出去
				 for(i = 0;i<text_head;i++)
				 {
						text_buf[i] = text_buff[i];
				 }
        return receive_date;				 
	 }
	 
	 //首位不为0 
	 else
	 {
	    //获得小数点前面的数据个数--前面已经做过数据标号何符号判断了，所以此处的head是第一个数据下标
		   i = 0;
		   head2 = 0;
		   date2 = 0;
		   date3 = 0;
		   while(date[head] !='.')
			 {
				     if(date[head] == '\0') return 0xfffff;//规避一下错误，此句可以不需要
			       head2+=1;
				     head +=1;			     
			 }
			 //倒回第一位数据，因为查找小数点时，加了head2；
			 head -=head2;
			 //---------------计算整数部分-------------
			 for(i = 0;i<head2;i++)
			 {
				 	//将字符保存
					  text_buff[text_head] = date[head+i];
					  text_head+=1;
				   //整数提取： 对应位加权
				 //列3:-567.124
				 // 5*10(2次方) 6*10（1次方） 7*10（0次方）
			     date2 = text_number(&date[head+i])*my_squre(10,head2-i-1);
					 date3 +=date2; 		 
			 }
	    //整数部分获取完成
			 receive_date = (float)date3;
			//--------------获取小数部分--------------
			 
		  //判断数据移位是否正确--避免前面移位错误则-加强数据的准确性
			 if(date[head+head2]=='.') signed_2 = 2;
			 //移位正确--接着获取小数部分
			 if(signed_2 == 2)
			 {
				 //加上小数点字符
				 text_buff[text_head] = '.';
				 text_head+=1;
				 
			    //小数数据个数
				    date_number  =  date_number -head2 -1; 
				 		head =head+head2+1;//直接定位到小数点后第一位数据
				    date2 = 0;
				    date3 = 0;
					for(i = 0;i<date_number;i++)
				 {
					   //保存字符
					   text_buff[text_head] = date[head+i];
					   text_head +=1;
					   //计算小数点后面数据
						 date2 = text_number(&date[head+i])*my_squre(10,date_number-i-1);
						 date3 +=date2; 				  
				 }
				 
			 }
			 //小数部分获取完成---注意不能使用int型数据做除法，否则只有输出整数
			 date1 = 0;	  
			 date1 = date3;
			 date1=date1/my_squre(10,date_number);
			 //加上小数部分
			 receive_date += date1;
			 //如果数据是负数，加上负号
			 if(signed_1 == 1) receive_date*=-1;
			 	 //将字符串传出去
		 for(i = 0;i<text_head;i++)
		 {
				text_buf[i] = text_buff[i];
		 }
	 }
	 
   return receive_date;

}

/*
函数功能：将浮点数据---->转为字符串
作者：何荣涛
参数：date  数据
函数使用注意：1-最大支持解析15个字符，如需要解析更大数据，则需要修改函数内部数组
              2-整数最大支持6位，小数六位，如需支持更大整数，需要修改代码


注意；STM32F407的float保留小数点后六位，且会有偏差
例  45.456     可能会偏差正负 0.000001
    -80.45679  可能会偏差正负 0.000001

*/
void date_number_text_float(float date,uint8_t *buff)
{
    int number = 0;
	  int number2 = 0;
	  int number3 = 0;
	  int number4 = 0;
	  uint8_t signed_1 = 0;
	  uint8_t signed_2 = 0;
	  uint8_t signed_3 = 0;
	  uint8_t i = 0;
	  uint8_t j = 0;
	  uint8_t buf[15];//修改此处数据个数，支持更大数据解析为字符串
	  *buff = 0;
		//判断数据是否为负数--如果为负数则字符第一位为符号--同时把数据变成正数处理
	   if(date<0)
		 {
			 date *=-1;
		   signed_1 = 1;
			 j+=1;
		 }
	  //将整数部分取出来
	  number = (int)date;

	  //判断整数部分是否为0，如果为0直接进行小数部分转换
	  signed_2 = 0;
		signed_3 = 0;
	  if(number ==0)signed_3 = 1;
    if(signed_3 != 1)
		{
    //处理整数部分--最大整数部分6位------修改此处代码兼容更大整数
	  for(i = 0;i<6;i++)
	   {
			 //判断首位数据是否为0
			  if(signed_2 ==0)
				{
					number2 = number/my_squre(10,5-i);//如需需改最大整数，此处也要修改
				}	
				//首位数据不为0
        else
				{
					 number2 = number%my_squre(10,5-i+1);//如需需改最大整数，此处也要修改
					 number2 = number2/my_squre(10,5-i);//如需需改最大整数，此处也要修改
				}
	      //判断数据	---之所以或上signed_2 保证第一位数据不为0,后面数据为0的情况，后面数据为0是有效数据			 
		    if(number2 !=0 || signed_2 ==1)
				{
					  signed_2 = 1;
				    switch(number2)
						{
							case 0:buf[j] = '0';break;
							case 1:buf[j] = '1';break;
							case 2:buf[j] = '2';break;
							case 3:buf[j] = '3';break;
							case 4:buf[j] = '4';break;
							case 5:buf[j] = '5';break;
							case 6:buf[j] = '6';break;
							case 7:buf[j] = '7';break;
							case 8:buf[j] = '8';break;
							case 9:buf[j] = '9';break;
						
						}
				    j+=1;		
				  }				
		 }
	 }
		//处理小数部分-----------------------
		//判断整数部分是否为0
		 if(signed_3 ==1)
		 {
			   //判断数据是否为负数，为负数则加上负号
			   if(signed_1 ==1)
				 {
					 buf[0] = '-';
					 buf[1] = '0';
					 buf[2] = '.';
					 j = 3;
				 }
				 else
				 {
					 buf[0] = '0';
					 buf[1] = '.';
					 j = 2;	 
				 }	
				 
		 }
		 //整数不为0的情况
		 else
		 {
			 //处理小数部分--减去整数部分，保留小数部分
			 date = date-(float)number;
			
			 //把小数点加进去
			 buf[j] = '.';
			 j+=1;
			 //上面使用到的数据清0，避免对下面数据提取影响
			 number2 = 0;	 
			 signed_2 = 0;
			 i = 0;
		 }
		 	 //直接将小数部分变成整数
      number3 = date*1000000.0f;

		 for(i = 0;i<6;i++)
		 {	     
			      number3 = number3 - number2;
			      number4 = number3 /my_squre(10,5-i);			 
			      number2 = number4*my_squre(10,5-i);
            			 
				    switch(number4)
						{
							case 0:buf[j] = '0';break;
							case 1:buf[j] = '1';break;
							case 2:buf[j] = '2';break;
							case 3:buf[j] = '3';break;
							case 4:buf[j] = '4';break;
							case 5:buf[j] = '5';break;
							case 6:buf[j] = '6';break;
							case 7:buf[j] = '7';break;
							case 8:buf[j] = '8';break;
							case 9:buf[j] = '9';break;					
						}
				    j+=1;	
					 //减去前面的整数，继续提取下一小数，这样就不用再去处理前面的整数
					//例 提取0.1234  
					//0.123*10 = 1.23，提取1  后面减去1得0.23 0.23*10 = 2.3 提取2 后面减去2 。。。					
		 }
		 //如果数据为负数，在这里加上负号，这里添加负号是保证整数不为0的情况为负数
		 if(signed_1 == 1)buf[0]= '-';
		 //将数传出去
		  for(i = 0;i<j;i++)
		 {
		    buff[i] = buf[i];
		 }	 
}


