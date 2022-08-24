#include<stdio.h>
#include<graphics.h>
#include<stdlib.h>//rand()
#include<time.h>//time()
#pragma comment(lib,"Winmm.lib")

//难度相关参数
#define fish_number 20
#define fish_chance 20
int stage_score[4] = { 100,500,1000,4000 };
int score[6] = { 20,50,80,100,200,1 };
int evolution = 10;

//全局变量
int count1 = 0;
int count2 = 0;//防暴死

//鱼的结构体
struct fish
{
	int x;
	int y;
	int width;
	int height;
	int check_width;
	int check_height;
	int speed; 
	int is_alive;
	int score;
	IMAGE* bg;
	IMAGE* self;
};
typedef struct fish FISH;
FISH fish[fish_number];

//玩家的结构体
struct player
{
	int x;
	int y;
	int width;
	int height;
	int direction; // 0:--> 1:<--
	int speed;
	int life;
	char Life[10];
	int score;
	char Score[10];
	IMAGE* bg;
	IMAGE* self;
};
typedef struct player PLAYER;
PLAYER player[2];

//图片内存申明
IMAGE* bg = new IMAGE;
IMAGE* intro = new IMAGE;
IMAGE* fish1bg_r = new IMAGE;
IMAGE* fish1_r = new IMAGE;
IMAGE* fish1bg_l = new IMAGE;
IMAGE* fish1_l = new IMAGE;
IMAGE* fish2bg_r = new IMAGE;
IMAGE* fish2_r = new IMAGE;
IMAGE* fish2bg_l = new IMAGE;
IMAGE* fish2_l = new IMAGE;
IMAGE* fish3bg_r = new IMAGE;
IMAGE* fish3_r = new IMAGE;
IMAGE* fish3bg_l = new IMAGE;
IMAGE* fish3_l = new IMAGE;
IMAGE* fish4bg_r = new IMAGE;
IMAGE* fish4_r = new IMAGE;
IMAGE* fish4bg_l = new IMAGE;
IMAGE* fish4_l = new IMAGE;
IMAGE* hippocampusbg_r = new IMAGE;
IMAGE* hippocampus_r = new IMAGE;
IMAGE* hippocampusbg_l = new IMAGE;
IMAGE* hippocampus_l = new IMAGE;
IMAGE* sharkbg_r = new IMAGE;
IMAGE* shark_r = new IMAGE;
IMAGE* sharkbg_l = new IMAGE;
IMAGE* shark_l = new IMAGE;

//显示文字于中心位置（x，y）
void showtext(int x, int y, const char arr[])
{
	outtextxy(x - textwidth(arr) / 2, y - textheight(arr) / 2, arr);
}

//显示带文字按钮于中心位置（x，y）
void showbutton(int x, int y, const char arr[])
{
	int bias = 10;
	fillrectangle(x - textwidth(arr) / 2 - bias, y - textheight(arr) / 2 - bias, x + textwidth(arr) / 2 + bias, y + textheight(arr) / 2 + bias);
	outtextxy(x - textwidth(arr) / 2, y - textheight(arr) / 2, arr);
	//调试：得到矩形尺寸
	//printf("%d,%d,%d,%d\n", x - textwidth(arr) / 2 - bias, y - textheight(arr) / 2 - bias, x + textwidth(arr) / 2 + bias, y + textheight(arr) / 2 + bias);
}

//music
void bgm(void)
{
	mciSendString("play ./bgm.mp3 repeat", NULL, 0, NULL);
}
void music_eat(void)
{
	mciSendString("play ./EAT.mp3", NULL, 0, NULL);
}
void music_hurt(void)
{
	mciSendString("play ./NOPE.mp3", NULL, 0, NULL);
}
void music_fail(void)
{
	mciSendString("play ./FAIL.mp3", NULL, 0, NULL);
}
void music_victory(void)
{
	mciSendString("play ./VICTORY.mp3", NULL, 0, NULL);
}

//添加一条鱼
void add_fish(int i)
{
	int p = rand();
	fish[i].is_alive = 1;
	fish[i].speed = p % 11 + 3;
	switch (p % 50 + 1)
	{
	case 1:
	case 2:
	case 3:
	case 14:
		fish[i].width = 40;
		fish[i].height = 20;
		fish[i].check_width = 30;
		fish[i].check_height = 15;
		fish[i].bg = fish1bg_l;
		fish[i].self = fish1_l;
		fish[i].x = 800;
		fish[i].speed = -fish[i].speed;
		fish[i].score = score[0];
		break;
	case 4:
	case 5:
	case 6:
		fish[i].width = 100;
		fish[i].height = 36;
		fish[i].check_width = 90;
		fish[i].check_height = 25;
		fish[i].bg = fish2bg_l;
		fish[i].self = fish2_l;
		fish[i].x = 800;
		fish[i].speed = -fish[i].speed;
		fish[i].score = score[1];
		break;
	case 7:
	case 8:
	case 9:
		fish[i].width = 120;
		fish[i].height = 70;
		fish[i].check_width = 110;
		fish[i].check_height = 40;
		fish[i].bg = fish3bg_l;
		fish[i].self = fish3_l;
		fish[i].x = 800;
		fish[i].speed = -fish[i].speed;
		fish[i].score = score[2];
		break;
	case 10:
	case 11:
	case 12:
		fish[i].width = 170;
		fish[i].height = 110;
		fish[i].check_width = 150;
		fish[i].check_height = 60;
		fish[i].bg = fish4bg_l;
		fish[i].self = fish4_l;
		fish[i].x = 800;
		fish[i].speed = -fish[i].speed;
		fish[i].score = score[3];
		break;
	case 13:
		fish[i].width = 486;
		fish[i].height = 164;
		fish[i].check_width = 460;
		fish[i].check_height = 100;
		fish[i].bg = sharkbg_l;
		fish[i].self = shark_l;
		fish[i].x = 800;
		fish[i].speed = -fish[i].speed;
		fish[i].score = score[4];
		break;
	case 15:
		fish[i].width = 40;
		fish[i].height = 80;
		fish[i].check_width = 40;
		fish[i].check_height = 80;
		fish[i].bg = hippocampusbg_l;
		fish[i].self = hippocampus_l;
		fish[i].x = 800;
		fish[i].speed = -fish[i].speed;
		fish[i].score = score[5];
		break;
	case 16:
	case 17:
	case 18:
	case 29:
		fish[i].width = 40;
		fish[i].height = 20;
		fish[i].check_width = 30;
		fish[i].check_height = 15;
		fish[i].bg = fish1bg_r;
		fish[i].self = fish1_r;
		fish[i].x = -fish[i].width;
		fish[i].speed = fish[i].speed;
		fish[i].score = score[0];
		break;
	case 19:
	case 20:
	case 21:
		fish[i].width = 100;
		fish[i].height = 36;
		fish[i].check_width = 90;
		fish[i].check_height = 25;
		fish[i].bg = fish2bg_r;
		fish[i].self = fish2_r;
		fish[i].x = -fish[i].width;
		fish[i].speed = fish[i].speed;
		fish[i].score = score[1];
		break;
	case 22:
	case 23:
	case 24:
		fish[i].width = 120;
		fish[i].height = 70;
		fish[i].check_width = 110;
		fish[i].check_height = 40;
		fish[i].bg = fish3bg_r;
		fish[i].self = fish3_r;
		fish[i].x = -fish[i].width;
		fish[i].speed = fish[i].speed;
		fish[i].score = score[2];
		break;
	case 25:
	case 26:
	case 27:
		fish[i].width = 170;
		fish[i].height = 110;
		fish[i].check_width = 150;
		fish[i].check_height = 60;
		fish[i].bg = fish4bg_r;
		fish[i].self = fish4_r;
		fish[i].x = -fish[i].width;
		fish[i].speed = fish[i].speed;
		fish[i].score = score[3];
		break;
	case 28:
		fish[i].width = 486;
		fish[i].height = 164;
		fish[i].check_width = 460;
		fish[i].check_height = 100;
		fish[i].bg = sharkbg_r;
		fish[i].self = shark_r;
		fish[i].x = -fish[i].width;
		fish[i].speed = fish[i].speed;
		fish[i].score = score[4];
		break;
	case 30:
		fish[i].width = 40;
		fish[i].height = 80;
		fish[i].check_width = 40;
		fish[i].check_height = 80;
		fish[i].bg = hippocampusbg_r;
		fish[i].self = hippocampus_r;
		fish[i].x = -fish[i].width;
		fish[i].speed = fish[i].speed;
		fish[i].score = score[5];
		break;
	case 31:
	case 32:
		fish[i].width = 40;
		fish[i].height = 20;
		fish[i].check_width = 30;
		fish[i].check_height = 15;
		fish[i].bg = fish1bg_l;
		fish[i].self = fish1_l;
		fish[i].x = 800;
		fish[i].speed = -fish[i].speed;
		fish[i].score = score[0];
		if (player[1].score + player[0].score < stage_score[0])break;
	case 33:
	case 34:
		fish[i].width = 100;
		fish[i].height = 36;
		fish[i].check_width = 90;
		fish[i].check_height = 25;
		fish[i].bg = fish2bg_l;
		fish[i].self = fish2_l;
		fish[i].x = 800;
		fish[i].speed = -fish[i].speed;
		fish[i].score = score[1];
		if (player[1].score + player[0].score < stage_score[1])break;
	case 35:
	case 36:
		fish[i].width = 120;
		fish[i].height = 70;
		fish[i].check_width = 110;
		fish[i].check_height = 40;
		fish[i].bg = fish3bg_l;
		fish[i].self = fish3_l;
		fish[i].x = 800;
		fish[i].speed = -fish[i].speed;
		fish[i].score = score[2];
		if (player[1].score + player[0].score < stage_score[2])break;
	case 37:
	case 38:
		fish[i].width = 170;
		fish[i].height = 110;
		fish[i].check_width = 150;
		fish[i].check_height = 60;
		fish[i].bg = fish4bg_l;
		fish[i].self = fish4_l;
		fish[i].x = 800;
		fish[i].speed = -fish[i].speed;
		fish[i].score = score[3];
		if (player[1].score + player[0].score < stage_score[3])break;
	case 39:
	case 40:
		fish[i].width = 486;
		fish[i].height = 164;
		fish[i].check_width = 460;
		fish[i].check_height = 100;
		fish[i].bg = sharkbg_l;
		fish[i].self = shark_l;
		fish[i].x = 800;
		fish[i].speed = fish[i].speed;
		fish[i].score = score[4];
		break;
	case 41:
	case 42:
		fish[i].width = 40;
		fish[i].height = 20;
		fish[i].check_width = 30;
		fish[i].check_height = 15;
		fish[i].bg = fish1bg_r;
		fish[i].self = fish1_r;
		fish[i].x = -fish[i].width;
		fish[i].speed = fish[i].speed;
		fish[i].score = score[0];
		if (player[1].score + player[0].score < stage_score[0])break;
	case 43:
	case 44:
		fish[i].width = 100;
		fish[i].height = 36;
		fish[i].check_width = 90;
		fish[i].check_height = 25;
		fish[i].bg = fish2bg_r;
		fish[i].self = fish2_r;
		fish[i].x = -fish[i].width;
		fish[i].speed = fish[i].speed;
		fish[i].score = score[1];
		if (player[1].score + player[0].score < stage_score[1])break;
	case 45:
	case 46:
		fish[i].width = 120;
		fish[i].height = 70;
		fish[i].check_width = 110;
		fish[i].check_height = 40;
		fish[i].bg = fish3bg_r;
		fish[i].self = fish3_r;
		fish[i].x = -fish[i].width;
		fish[i].speed = fish[i].speed;
		fish[i].score = score[2];
		if (player[1].score + player[0].score < stage_score[2])break;
	case 47:
	case 48:
		fish[i].width = 170;
		fish[i].height = 110;
		fish[i].check_width = 150;
		fish[i].check_height = 60;
		fish[i].bg = fish4bg_r;
		fish[i].self = fish4_r;
		fish[i].x = -fish[i].width;
		fish[i].speed = -fish[i].speed;
		fish[i].score = score[3];
		if (player[1].score + player[0].score < stage_score[3])break;
	case 49:
	case 50:
		fish[i].width = 486;
		fish[i].height = 164;
		fish[i].check_width = 460;
		fish[i].check_height = 100;
		fish[i].bg = sharkbg_r;
		fish[i].self = shark_r;
		fish[i].x = -fish[i].width;
		fish[i].speed = fish[i].speed;
		fish[i].score = score[4];
		break;
	}
	fish[i].y = p % (600 - fish[i].height*2) + 50;	
}

//添加一位玩家
void add_player(int i,int x,int y, int direction)
{
	player[i].x = x;
	player[i].y = y;
	player[i].width = 55;
	player[i].height = 35;
	player[i].direction = direction;
	player[i].speed = 10;
	player[i].life = 5;
	_itoa_s(player[i].life, player[i].Life,10);
	player[i].score = 0;
	_itoa_s(player[i].score, player[i].Score, 10);
	printf("Player[%d] is born!\n", i);
}

//显示玩家
void show_player(int i)
{
	if (player[i].life > 0)
	{
		if (player[i].direction == 0)
		{
			if (i == 0)
			{
				IMAGE* player1bg_r = new IMAGE;
				IMAGE* player1_r = new IMAGE;
				loadimage(player1bg_r, "./player1bg_r.jpg", player[i].width, player[i].height);
				loadimage(player1_r, "./player1_r.jpg", player[i].width, player[i].height);
				player[i].bg = player1bg_r;
				player[i].self = player1_r;
				putimage(player[i].x, player[i].y, player[i].width, player[i].height, player[i].bg, 0, 0, SRCAND);
				putimage(player[i].x, player[i].y, player[i].width, player[i].height, player[i].self, 0, 0, SRCINVERT);
				delete player1bg_r;
				delete player1_r;
			}
			if (i == 1)
			{
				IMAGE* player2bg_r = new IMAGE;
				IMAGE* player2_r = new IMAGE;
				loadimage(player2bg_r, "./player2bg_r.jpg", player[i].width, player[i].height);
				loadimage(player2_r, "./player2_r.jpg", player[i].width, player[i].height);
				player[i].bg = player2bg_r;
				player[i].self = player2_r;
				putimage(player[i].x, player[i].y, player[i].width, player[i].height, player[i].bg, 0, 0, SRCAND);
				putimage(player[i].x, player[i].y, player[i].width, player[i].height, player[i].self, 0, 0, SRCINVERT);
				delete player2bg_r;
				delete player2_r;
			}
		}
		if (player[i].direction == 1)
		{
			if (i == 0)
			{
				IMAGE* player1bg_l = new IMAGE;
				IMAGE* player1_l = new IMAGE;
				loadimage(player1bg_l, "./player1bg_l.jpg", player[i].width, player[i].height);
				loadimage(player1_l, "./player1_l.jpg", player[i].width, player[i].height);
				player[i].bg = player1bg_l;
				player[i].self = player1_l;
				putimage(player[i].x, player[i].y, player[i].width, player[i].height, player[i].bg, 0, 0, SRCAND);
				putimage(player[i].x, player[i].y, player[i].width, player[i].height, player[i].self, 0, 0, SRCINVERT);
				delete player1bg_l;
				delete player1_l;
			}
			if (i == 1)
			{
				IMAGE* player2bg_l = new IMAGE;
				IMAGE* player2_l = new IMAGE;
				loadimage(player2bg_l, "./player2bg_l.jpg", player[i].width, player[i].height);
				loadimage(player2_l, "./player2_l.jpg", player[i].width, player[i].height);
				player[i].bg = player2bg_l;
				player[i].self = player2_l;
				putimage(player[i].x, player[i].y, player[i].width, player[i].height, player[i].bg, 0, 0, SRCAND);
				putimage(player[i].x, player[i].y, player[i].width, player[i].height, player[i].self, 0, 0, SRCINVERT);
				delete player2bg_l;
				delete player2_l;
			}
		}
		//printf("width=%d height=%d\n", player[i].width, player[i].height);
	}
}

//显示积分
void show_score(int i)
{
	_itoa_s(player[i].life, player[i].Life, 10);
	_itoa_s(player[i].score, player[i].Score, 10);
	{
		if (i == 0)
		{
			showtext(100, 30, player[i].Life);
			showtext(330, 30, player[i].Score);
		}
		if (i == 1)
		{
			showtext(500, 30, player[i].Life);
			showtext(730, 30, player[i].Score);
		}
	}
}

//显示鱼
void show_fish(void)
{
	for (int i = 0; i < fish_number; i++)
	{
		if (fish[i].is_alive == 1)
		{
			fish[i].x += fish[i].speed;
			if(fish[i].x<-fish[i].width || fish[i].x>800)
			{
				fish[i].is_alive = 0;
			}
			else
			{
				putimage(fish[i].x, fish[i].y, fish[i].bg, SRCAND);
				putimage(fish[i].x, fish[i].y, fish[i].self, SRCINVERT);
			}
		}
	}
}

//鱼的随机生成
void born_fish(void)
{
	int p = rand();
	for (int i = 0; i < fish_number; i++)
	{
		if (p % fish_chance == 1 && fish[i].is_alive == 0)
		{
			p++;
			add_fish(i);
			printf("Fish[%d] is born!\n", i);
		}
	}
}

//碰撞判断
void check_fish(void)
{
	for (int i = 0; i < fish_number; i++)
	{
		if (fish[i].is_alive == 1)
		{
			if (player[0].life > 0)
			{
				if (abs(fish[i].x + fish[i].width / 2 - player[0].x - player[0].width / 2) < (fish[i].check_width + player[0].width) / 2 
					&& abs(fish[i].y + fish[i].height / 2 - player[0].y - player[0].height / 2) < (fish[i].check_height + player[0].height) / 2)
				{
					if (fish[i].check_width * fish[i].check_height < player[0].width * player[0].height)
					{
						music_eat();
						fish[i].is_alive = 0;
						if (fish[i].score == 1)
						{
							if (player[0].score < 2000)
							{
								player[0].score += 400;
							}
							else
							{
								player[0].score *= 1.05;
							}
						}
						else
						{
							player[0].score += fish[i].score;
						}
					}
					else if (count1 > 30)
					{
						music_hurt();
						player[0].life -= 1;
						count1 = 0;
						printf("player[0].life = %d\n", player[0].life);
					}
				}
			}
			if (player[1].life > 0)
			{
				if (abs(fish[i].x + fish[i].width / 2 - player[1].x - player[1].width / 2) < (fish[i].width + player[1].width) / 2
					&& abs(fish[i].y + fish[i].height / 2 - player[1].y - player[1].height / 2) < (fish[i].height + player[1].height) / 2)
				{
					if (fish[i].width * fish[i].height < player[1].width * player[1].height)
					{
						music_eat();
						fish[i].is_alive = 0;
						if (fish[i].score == 1)
						{
							if (player[1].score < 5000)
							{
								player[1].score += 1000;
							}
							else
							{
								player[1].score *= 2;
							}
						}
						else
						{
							player[1].score += fish[i].score;
						}
					}
					else if(count2 > 30)
					{
						music_hurt();
						player[1].life -= 1;
						count2 = 0;
						printf("player[1].life = %d\n", player[1].life);
					}
				}
			}
		}
	}
}

//玩家成长判断
void check_player(int i)
{
	static int last_score1=0;
	static int count1=0;
	count1 += (player[0].score - last_score1) / 10;
	if (count1 >= evolution)
	{
		count1 -= evolution;
		player[0].width += 11;
		player[0].height += 7;
	}
	last_score1 = player[0].score;
	if (i == 1)
	{
		static int last_score2 = 0;
		static int count2 = 0;
		count2 += (player[1].score - last_score2) / 10;
		if (count2 >= evolution)
		{
			count2 -= evolution;
			player[1].width += 11;
			player[1].height += 7;
		}
		last_score2 = player[1].score;
	}
}

//菜单
void menu(void) 
{
	BeginBatchDraw();

	setbkmode(TRANSPARENT);
	loadimage(bg, "./bg.jpg");
	putimage(0, 0, bg);

	loadimage(intro, "./intro0.jpg");
	putimage(300 - 377 / 2, 320 - 232 / 2, intro);

	settextstyle(50, 0, "黑体");
	settextcolor(BLUE);
	showtext(400, 100, "大鱼吃小鱼双人版");

	setlinestyle(PS_SOLID, 3);
	setlinecolor(BLACK);
	setfillcolor(RGB(0, 204, 255));
	settextcolor(BLUE);
	settextstyle(28, 0, "黑体");
	showbutton(600, 250, "开始游戏");
	showbutton(600, 320, "操作说明");
	showbutton(600, 390, "游戏介绍");

	ExMessage msg;
	while (1)
	{
		if (peekmessage(&msg, EM_MOUSE))
		{
			if (msg.x > 534 && msg.x < 666 && msg.y > 226 && msg.y < 274)
			{
				if (msg.message == WM_LBUTTONDOWN)
				{
					printf("开始游戏\n");
					break;
				}
				else
				{
					setfillcolor(RGB(0, 255, 255));
					showbutton(602, 252, "开始游戏");
				}
			}
			else if (msg.x > 534 && msg.x < 666 && msg.y > 296 && msg.y < 344)
			{
				if (msg.message == WM_LBUTTONDOWN)
				{
					printf("操作说明\n");
					loadimage(intro, "./intro2.jpg");
					putimage(300 - 377 / 2, 320 - 232 / 2, intro);
				}
				else
				{
					setfillcolor(RGB(0, 255, 255));
					showbutton(602, 322, "操作说明");
				}
			}
			else if (msg.x > 534 && msg.x < 666 && msg.y > 366 && msg.y < 414)
			{
				if (msg.message == WM_LBUTTONDOWN)
				{
					printf("游戏介绍\n");
					loadimage(intro, "./intro1.jpg");
					putimage(300 - 377 / 2, 320 - 232 / 2, intro);
				}
				else
				{
					setfillcolor(RGB(0, 255, 255));
					showbutton(602, 392, "游戏介绍");
				}
			}
			else
			{
				putimage(0, 0, bg);

				IMAGE *intro = new IMAGE;
				loadimage(intro, "./intro0.jpg");
				putimage(300 - 377 / 2, 320 - 232 / 2, intro);
				delete intro;

				settextstyle(50, 0, "黑体");
				settextcolor(BLUE);
				showtext(400, 100, "大鱼吃小鱼双人版");

				setlinestyle(PS_SOLID, 3);
				setlinecolor(BLACK);
				setfillcolor(RGB(0, 204, 255));
				settextcolor(BLUE);
				settextstyle(28, 0, "黑体");
				showbutton(600, 250, "开始游戏");
				showbutton(600, 320, "操作说明");
				showbutton(600, 390, "游戏介绍");
			}
			FlushBatchDraw();
		}
	}
	cleardevice();
	EndBatchDraw();
}

//选择单人或双人
int choice(void)
{
	int player = 1;

	BeginBatchDraw();

	setbkmode(TRANSPARENT);
	IMAGE bg;
	loadimage(&bg, "./bg.jpg");
	putimage(0, 0, &bg);

	settextstyle(50, 0, "黑体");
	settextcolor(BLUE);
	setlinestyle(PS_SOLID, 3);
	setlinecolor(BLACK);
	setfillcolor(RGB(0, 204, 255));
	showtext(400, 100, "大鱼吃小鱼双人版");
	showbutton(400, 250, "单人游戏");
	showbutton(400, 350, "双人游戏");
	
	ExMessage msg;
	while (1)
	{
		if (peekmessage(&msg, EM_MOUSE))
		{
			if (msg.x > 290 && msg.x < 510 && msg.y > 215 && msg.y < 285)
			{
				if (msg.message == WM_LBUTTONDOWN)
				{
					printf("单人游戏\n");
					break;
				}
				else
				{
					setfillcolor(RGB(0, 255, 255));
					showbutton(402, 252, "单人游戏");
				}
			}
			else if (msg.x > 290 && msg.x < 510 && msg.y > 315 && msg.y < 385)
			{
				if (msg.message == WM_LBUTTONDOWN)
				{
					player = 2;
					printf("双人游戏\n");
					break;
				}
				else
				{
					setfillcolor(RGB(0, 255, 255));
					showbutton(402, 352, "双人游戏");
				}
			}
			else
			{
				setbkmode(TRANSPARENT);
				IMAGE bg;
				loadimage(&bg, "./bg.jpg");
				putimage(0, 0, &bg);

				settextstyle(50, 0, "黑体");
				settextcolor(BLUE);
				setlinestyle(PS_SOLID, 3);
				setlinecolor(BLACK);
				setfillcolor(RGB(0, 204, 255));
				showtext(400, 100, "大鱼吃小鱼双人版");
				showbutton(400, 250, "单人游戏");
				showbutton(400, 350, "双人游戏");
			}
			FlushBatchDraw();
		}
	}
	EndBatchDraw();
	return player;
}

//玩家初始化
void playerinit(void)
{
	player[0].life = 0;
	player[1].life = 0;
	player[0].width = 55;
	player[1].width = 55;
}

//游戏初始化
void gameinit(int player)
{
	srand((unsigned)time(NULL));
	
	playerinit();
	add_player(0, 300, 300, 0);
	if (player == 2)
	{
		add_player(1, 400, 300, 0);
	}
	
	for (int i = 0; i < 20; i++)
	{
		fish[i].is_alive = 0;
	}

	loadimage(bg, "./bg.jpg");
	loadimage(fish1bg_r, "./fish1bg_r.jpg");
	loadimage(fish1_r, "./fish1_r.jpg");
	loadimage(fish1bg_l, "./fish1bg_l.jpg");
	loadimage(fish1_l, "./fish1_l.jpg");
	loadimage(fish2bg_r, "./fish2bg_r.jpg");
	loadimage(fish2_r, "./fish2_r.jpg");
	loadimage(fish2bg_l, "./fish2bg_l.jpg");
	loadimage(fish2_l, "./fish2_l.jpg");
	loadimage(fish3bg_r, "./fish3bg_r.jpg");
	loadimage(fish3_r, "./fish3_r.jpg");
	loadimage(fish3bg_l, "./fish3bg_l.jpg");
	loadimage(fish3_l, "./fish3_l.jpg");
	loadimage(fish4bg_r, "./fish4bg_r.jpg");
	loadimage(fish4_r, "./fish4_r.jpg");
	loadimage(fish4bg_l, "./fish4bg_l.jpg");
	loadimage(fish4_l, "./fish4_l.jpg");
	loadimage(sharkbg_r, "./sharkbg_r.jpg");
	loadimage(shark_r, "./shark_r.jpg");
	loadimage(sharkbg_l, "./sharkbg_l.jpg");
	loadimage(shark_l, "./shark_l.jpg");
	loadimage(hippocampusbg_r, "./hippocampusbg_r.jpg");
	loadimage(hippocampus_r, "./hippocampus_r.jpg");
	loadimage(hippocampusbg_l, "./hippocampusbg_l.jpg");
	loadimage(hippocampus_l, "./hippocampus_l.jpg");

	printf("初始化成功\n");
}

//玩家操作
void keyboard(int i)
{
	if (i == 0)
	{
		if (GetAsyncKeyState(VK_RIGHT))
		{
			player[0].direction = 0;
			player[0].x += player[0].speed;
			printf("PLAYER1 GO RIGHT\n");
		}
		if (GetAsyncKeyState(VK_LEFT))
		{
			player[0].direction = 1;
			player[0].x -= player[0].speed; 
			printf("PLAYER1 GO LEFT\n");
		}
		if (GetAsyncKeyState(VK_UP))
		{
			player[0].y -= player[0].speed;
			printf("PLAYER1 GO UP\n");
		}
		if (GetAsyncKeyState(VK_DOWN))
		{
			player[0].y += player[0].speed;
			printf("PLAYER1 GO DOWN\n");
		}
		if (player[0].y < 0)
		{
			player[0].y = 0;
		}
		if (player[0].y > 600 - player[0].height)
		{
			player[0].y = 600 - player[0].height;
		}
		if (player[0].x < 0)
		{
			player[0].x = 0;
		}
		if (player[0].x > 800 - player[0].width)
		{
			player[0].x = 800 - player[0].width;
		}
	}
	if (i == 1)
	{
		if (GetAsyncKeyState(0x44))//'d'
		{
			player[1].direction = 0;
			player[1].x += player[1].speed;
			printf("PLAYER1 GO RIGHT\n");
		}
		if (GetAsyncKeyState(0x41))//'a'
		{
			player[1].direction = 1;
			player[1].x -= player[1].speed;
			printf("PLAYER1 GO LEFT\n");
		}
		if (GetAsyncKeyState(0x57))//'w'
		{
			player[1].y -= player[1].speed;
			printf("PLAYER1 GO UP\n");
		}
		if (GetAsyncKeyState(0x53))//'s'
		{
			player[1].y += player[1].speed;
			printf("PLAYER1 GO DOWN\n");
		}
		if (player[1].y < 0)
		{
			player[1].y = 0;
		}
		if (player[1].y > 600 - player[1].height)
		{
			player[1].y = 600 - player[1].height;
		}
		if (player[1].x < 0)
		{
			player[1].x = 0;
		}
		if (player[1].x > 800 - player[1].width)
		{
			player[1].x = 800 - player[1].width;
		}
	}
}

//游戏是否结束
int check_gameover(void)
{
	if (player[0].life + player[1].life == 0)
	{
		music_fail();
		return 1;
	}
	else if (player[0].width > 600 || player[1].width > 600)
	{
		music_victory();
		return 1;
	}
	else
	{
		return 0;
	}
}

//游戏界面
void game(int player)
{
	//游戏初始化
	gameinit(player);
	
	//游戏
	BeginBatchDraw();
	while(1)
	{
		//鱼的随机生成
		born_fish();

		//玩家的操控
		keyboard(0);
		if (player == 2)
		{
			keyboard(1);
		}

		//玩家成长判断
		check_player(0);
		if(player == 2)
		{
			check_player(1);
		}

		//碰撞判断
		check_fish();
		count1++;
		if (player == 2)
		{
			count2++;
		}

		//游戏是否结束
		if (check_gameover())
		{
			break;
		}
		
		//界面
		setbkmode(TRANSPARENT);
		putimage(0, 0, bg);
		settextstyle(50, 0, "Consolas");
		settextcolor(RED);
		showtext(30, 30, "1P");
		showtext(230, 30, "SCORE:");
		if (player == 2)
		{
			showtext(430, 30, "2P");
			showtext(630, 30, "SCORE:");
		}

		//显示鱼
		show_fish();

		//显示玩家
		show_player(0);
		if (player == 2)
		{
			show_player(1);
		}

		//显示积分
		show_score(0);
		if (player == 2)
		{
			show_score(1);
		}

		FlushBatchDraw();
		Sleep(30);
		cleardevice();
	}
	EndBatchDraw();
}

//游戏结束界面
void gameover(void)
{
	BeginBatchDraw();

	setbkmode(TRANSPARENT);
	loadimage(bg, "./bg.jpg");
	putimage(0, 0, bg);

	settextstyle(100, 0, "黑体");
	settextcolor(BLUE);
	showtext(400, 100, "YOUR SCORE");

	char Score[10];
	_itoa_s(player[0].score+player[1].score, Score, 10);
	showtext(400, 250, Score);

	setlinestyle(PS_SOLID, 3);
	setlinecolor(BLACK);
	setfillcolor(RGB(0, 204, 255));
	settextcolor(BLUE);
	settextstyle(28, 0, "黑体");
	showbutton(400,400,"重新游戏");

	ExMessage msg;
	while (1)
	{
		if (peekmessage(&msg, EM_MOUSE))
		{
			if (msg.x > 334 && msg.x < 466 && msg.y > 376 && msg.y < 424)
			{
				if (msg.message == WM_LBUTTONDOWN)
				{
					printf("重新游戏\n");
					break;
				}
				else
				{
					setfillcolor(RGB(0, 255, 255));
					showbutton(402, 402, "重新游戏");
				}
			}
			else
			{
				putimage(0, 0, bg);

				settextstyle(100, 0, "黑体");
				settextcolor(BLUE);
				showtext(400, 100, "YOUR SCORE");
				showtext(400, 250, Score);

				setlinestyle(PS_SOLID, 3);
				setlinecolor(BLACK);
				setfillcolor(RGB(0, 204, 255));
				settextcolor(BLUE);
				settextstyle(28, 0, "黑体");
				showbutton(400, 400, "重新游戏");
			}
			FlushBatchDraw();
		}
	}
	EndBatchDraw();
	cleardevice();
}

int main(void)
{
	//游戏窗口初始化
	//initgraph(800, 600, SHOWCONSOLE);
	initgraph(800, 600);
	bgm();

	while(1) 
	{
		//菜单
		menu();

		//游戏界面
		game(choice());

		//游戏结束界面
		gameover();
	}

	closegraph();
	return 0;
}
