
#include "vote_locate2.h"


Cvote_locate2::Cvote_locate2()
{
}


Cvote_locate2::~Cvote_locate2()
{
}


void Cvote_locate2::Vote(int num, std::vector<std::vector<std::vector<int>>>& vote, int** &mesh_vote, double * angle, std::vector<cv::Point3d> MapArray_)
{
	for (size_t i = 0; i < num; i++)//ѭ�����е�ƥ���ԣ��������������ͶƱ
	{
		if (angle[i] == pi / 2 || angle[i] == 1.5*pi || angle[i] == -pi / 2 || angle[i] == -1.5*pi)//��ֱ��б�ʲ�����ʱ ֱ��x=a
		{
			if (MapArray_.at(i).x - int(MapArray_.at(i).x) == 0 || MapArray_.at(i).x>0) //���﷽��XΪ�������Ҵ���0ʱ
			{
				for (size_t j = 0; j < 7; j++)
				{
					vote[j][int(MapArray_.at(i).x)].push_back(i);
					vote[j][int(MapArray_.at(i).x) - 1].push_back(i);

					mesh_vote[j][int(MapArray_.at(i).x)]++;//�������ĸ����Լ�
					mesh_vote[j][int(MapArray_.at(i).x) - 1]++;

				}
			}
			else//�﷽��X��Ϊ����ʱ
			{
				for (size_t j = 0; j < 7; j++)
				{
					vote[j][int(MapArray_.at(i).x)].push_back(i);
					mesh_vote[j][int(MapArray_.at(i).x)]++;//�������ĸ����Լ�
				}
			}
		}
		if (angle[i] == pi || angle[i] == 2 * pi || angle[i] == -pi)//ֱ��б��Ϊ0 
		{
			if (MapArray_.at(i).y - int(MapArray_.at(i).y) == 0 || MapArray_.at(i).y>0)//�﷽��YֵΪ�������Ҵ���0ʱ
			{
				for (size_t j = 0; j < 15; j++)
				{
					vote[int(MapArray_.at(i).y)][j].push_back(i);
					vote[int(MapArray_.at(i).y) - 1][j].push_back(i);

					mesh_vote[int(MapArray_.at(i).y)][j]++;//�������ĸ����Լ�
					mesh_vote[int(MapArray_.at(i).y) - 1][j]++;
				}
			}
			else//�﷽��Yֵ��Ϊ����
			{
				for (size_t j = 0; j < 15; j++)
				{
					vote[int(MapArray_.at(i).y)][j].push_back(i);
					mesh_vote[int(MapArray_.at(i).y)][j]++;//�������ĸ����Լ�
				}
			}
		}
		if ((angle[i]>0 && angle[i]<pi / 2) || (angle[i]>pi&&angle[i]<1.5*pi) || (angle[i]>-pi&&angle[i]<-0.5*pi) || (angle[i]>-2 * pi&&angle[i]<-1.5*pi))//ֱ��б�ʴ���0
		{
			for (size_t j = 0; j < 15; j++)//�ȱ���y=f(n)��nΪ����
			{
				double y = tan(angle[i])*(j - MapArray_.at(i).x) + MapArray_.at(i).y;
				if (y >= 0 && y<6.99)
				{
					vote[int(y)][j].push_back(i);
					mesh_vote[int(y)][j]++;//�������ĸ����Լ�
				}
			}

			for (size_t j = 0; j < 7; j++)
			{
				double x = (j - MapArray_.at(i).y) / tan(angle[i]) + MapArray_.at(i).x;
				if (x >= 0 && x<14.55)
				{

					vote[j][int(x)].push_back(i);
					mesh_vote[j][int(x)]++;//�������ĸ����Լ�
				}
			}
		}
		if ((angle[i] > pi / 2 && angle[i] < pi) || (angle[i] > -1.5*pi&&angle[i] < -pi) || (angle[i]>-pi / 2 && angle[i]<0))//ֱ�ߵ�б��С��0
		{
			for (size_t j = 0; j < 15; j++)
			{
				double y = tan(angle[i])*(j - MapArray_.at(i).x) + MapArray_.at(i).y;
				if (y>0 && y<6.99)
				{
					vote[int(y)][j].push_back(i);
					mesh_vote[int(y)][j]++;//�������ĸ����Լ�
				}
			}
			for (size_t j = 1; j < 6; j++)
			{
				double x = (j - MapArray_.at(i).y) / tan(angle[i]) + MapArray_.at(i).x;
				if (x >= 0 && x<14.55)
				{
					vote[j - 1][int(x)].push_back(i);
					mesh_vote[j - 1][int(x)]++;//�������ĸ����Լ�
				}
			}
			double x_ymax = (6.99 - MapArray_.at(i).y) / tan(angle[i]) + MapArray_.at(i).x;
			if (x_ymax>0 && x_ymax<6.99)
			{
				vote[6][int(x_ymax)].push_back(i);
				mesh_vote[6][int(x_ymax)]++;
			}
		}
	}
}


void Cvote_locate2::Locate2d(int num, std::vector<double> angle, std::vector<cv::Point2d> MapArray, double &X, double &Y)//ֱ���ཻ�������λ��
{
	double** A;//ϵ������
	double* b;//����
	double ATA[2][2] = { 0 };
	double ATb[2][1] = { 0 };
	double detATA;//���������ʽ
	double invATA[2][2];//�������

	A = (double**)malloc(sizeof(double*)*num);//����ռ�
	for (size_t i = 0; i < num; i++)
	{
		A[i] = (double*)malloc(sizeof(double) * 2);
	}
	b = (double*)malloc(sizeof(double)*num);


	for (size_t i = 0; i < num; i++) //����ϵ������A,����b
	{
		if (angle[i] == pi / 2 || angle[i] == 3 / 2 * pi)// ֱ��б��Ϊ0ʱ
		{
			A[i][0] = 1;
			A[i][1] = 0;
			b[i] = MapArray.at(i).x;
		}
		else//б�ʲ�Ϊ0ʱ
		{
			A[i][0] = -tan(angle[i]);
			A[i][1] = 1;
			b[i] = -tan(angle[i])*MapArray.at(i).x + MapArray.at(i).y;
		}
	}

	for (size_t i = 0; i < 2; i++)  //����ATA
	{
		for (size_t j = 0; j < 2; j++)
		{
			for (size_t k = 0; k < num; k++)
			{
				ATA[i][j] = ATA[i][j] + A[k][i] * A[k][j];
			}
		}
	}

	for (size_t i = 0; i < 2; i++)//����ATb
	{
		for (size_t k = 0; k < num; k++)
		{
			ATb[i][0] = ATb[i][0] + A[k][i] * b[k];
		}
	}

	detATA = ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0]; //����ATA����
	invATA[0][0] = ATA[1][1] / detATA;
	invATA[0][1] = -ATA[1][0] / detATA;
	invATA[1][0] = -ATA[0][1] / detATA;
	invATA[1][1] = ATA[0][0] / detATA;

	X = invATA[0][0] * ATb[0][0] + invATA[0][1] * ATb[1][0];
	Y = invATA[1][0] * ATb[0][0] + invATA[1][1] * ATb[1][0];
//	std::cout << "��λ�����" << std::endl << "X:" << X << "\t" << "Y:" << Y << std::endl;
	std::cout << "result" << std::endl << "X:" << X << "\t" << "Y:" << Y << std::endl;
}

double* Cvote_locate2::culAngle(int width, int f, double* angle, const std::vector<cv::Point2d> MarkArray_, double oriention) //���������࣬�Ƕȣ����,���򴫸���������
{

	for (size_t i = 0; i < MarkArray_.size(); i++)
	{
		double a = atan((width / 2 - MarkArray_.at(i).x) / f);
		angle[i] = a - oriention + deltaA;  //�������﷽��ά����ϵ��ƥ�������ڵķ���
	}

	return angle;
}

bool Cvote_locate2::run(cv::Mat mK, std::vector<cv::Point2d>& MarkArray_, std::vector<cv::Point3d>& MapArray_,
	double &X, double &Y, double Angle_or) {

	//Angle_or = 310.66998;
	//Angle_or = 220.66998;
	//Angle_or = 40.66998;
	//Angle_or = 130.66998;
	//Angle_or = pi * Angle_or / 180;

	//MapArray_Ϊƥ���Ե��﷽������
	//MarkArray_Ϊƥ���Ե��񷽶�ά����
	int num = MarkArray_.size();//��ʼƥ������



	int width =2*mK.at<double>(0,2) ;
	int focal =mK.at<double>(0,0);
	//width = 2976;
	//focal = 3500;
	//int height = 3968;



	double *angle;//����ƥ���Թ��ߵĽǶ�
	angle = (double*)malloc(sizeof(double)*num);
	angle = culAngle(width, focal, angle, MarkArray_, Angle_or); //����culAngle����������ÿһ�����ߵĽǶ�

	std::vector<cv::Point2d> MapArray;//ɸѡ����﷽���ƽ������
	std::vector<double> Angle;//ɸѡ��ĽǶ�

	std::vector<int> a;
	std::vector<std::vector<std::vector<int>>> vote(7, std::vector<std::vector<int>>(15, a));//����1*1��������άvector�ĵ���ά��ƥ����

	int** mesh_vote;//����һ�������¼ÿ�����Ӵ�����ֱ�߸���
	mesh_vote = (int**)malloc(sizeof(int*) * 7);
	for (size_t i = 0; i < 7; i++)
	{
		mesh_vote[i] = (int*)malloc(sizeof(int) * 15);
	}
	for (size_t i = 0; i < 7; i++)
	{
		for (size_t j = 0; j < 15; j++)
		{
			mesh_vote[i][j] = 0;
		}
	}


	Vote(num, vote, mesh_vote, angle, MapArray_);


	int max = 0;
	int row =0, col=0;//��¼��������ߵ����������

	for (size_t i = 0; i < 7; i++) //�ҳ�ͶƱ����������
	{
		for (size_t j = 0; j < 15; j++)
		{
			if (mesh_vote[i][j]>max)
			{
				max = mesh_vote[i][j];
				row = i;
				col = j;
			}
		}
	}

	int pairNum = mesh_vote[row][col];//pairNum�м�¼��ȷƥ��Եĸ�����mesh_vote[row][col]ΪͶƱ��������ͶƱ��

	std::cout << "ͶƱ��ƥ��������" << pairNum << " row " << row << " col " << col  <<std::endl;

	//std::ofstream out1("2d.txt");//������ڼ�����������
	//std::ofstream out2("3d.txt");//������ڼ�����������
	std::ofstream out("2d-3d.txt");
	std::ofstream outimgpoint("2dhugin.txt");
	std::ofstream outangle("angle.txt");


	for (size_t i = 0; i < pairNum; i++)//�õ���ȷƥ��ĵ�Ժ���ȷƥ��ĽǶ�
	{
		int pt_id = vote[row][col][i];
		cv::Point2d pt2d;//�﷽���XY

		pt2d.x = MapArray_.at(pt_id).x;
		pt2d.y = MapArray_.at(pt_id).y;
		MapArray.push_back(pt2d);
		Angle.push_back(angle[pt_id]);

		cv::Point2d p2d;//�������
		cv::Point3d p3d;//�﷽������
		double tempangle;//�Ƕ�
		tempangle = angle[pt_id];
		p2d.x = MarkArray_.at(pt_id).x;
		p2d.y = MarkArray_.at(pt_id).y;
		p3d.x = MapArray_.at(pt_id).x;
		p3d.y = MapArray_.at(pt_id).y;
		p3d.z = MapArray_.at(pt_id).z;
		//out1 << p2d.x << "\t" << p2d.y << "\n";//������ڼ���λ�õ����
		//out2 << p3d.x << "\t" << p3d.y << "\t" << p3d.z << "\n";//������ڼ���λ�õ����
		out << p2d.x << "\t" << p2d.y << "\t\t" << p3d.x << "\t" << p3d.y << "\t" << p3d.z << "\n";//����������
																								   //c n0 N0 x1234.33 y789.89 X1234.33 Y789.89 t0
		outimgpoint << "c n0 N0 x" << p2d.x << " y" << p2d.y << " X" << p2d.x << " Y" << p2d.y << " t0" << "\n";//���hugin���ģʽ
		outangle << tempangle << "\n";


	}

	Locate2d(pairNum, Angle, MapArray, X, Y);

	//cv::waitKey(0);
	return 0;

}

