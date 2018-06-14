#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <vector>
#include <optional>
#include <algorithm>
#include <random>
#include <omp.h>
#include <time.h>
#include <sstream>

// Vector3
struct V {
	double x;
	double y;
	double z;

	V(double v = 0) : V(v,v,v) {}
	V(double x, double y, double z) : x(x), y(y), z(z) {}
	double operator[](int i) const { return (&x)[i]; }
};

// Vector3 Operator
V operator+(V a, V b) {
	return V(a.x + b.x, a.y + b.y, a.z + b.z);
}

V operator-(V a, V b) {
	return V(a.x - b.x, a.y - b.y, a.z - b.z);
}

V operator*(V a, V b) {
	return V(a.x * b.x, a.y * b.y, a.z * b.z);
}

V operator/(V a, V b) {
	return V(a.x / b.x, a.y / b.y, a.z / b.z);
}

V operator-(V v) {
	return V(-v.x, -v.y, -v.z);
}

double dot(V a, V b){
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

V cross(V a, V b) {
	return V(
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	);
}

V normarize(V v) {
	return v / sqrt(dot(v, v));
}

// tangentSpace
std::tuple<V, V> tangentSpace(const V& n) {
	const double s = std::copysign(1, n.z);
	const double a = -1 / (s + n.z);
	const double b = n.x * n.y * a;
	return{
		V(1 + s * n.x * n.x * a, s * b, -s * n.x),
		V(b,s + n.y * n.y * a, -n.y)
	};
}

//Tone Map
int tonemap(double v) {
	// �K���}�␳�������Ă��� 0 - 255 �ŃN�����v����
	return std::min(std::max(int(std::pow(v, 1 / 2.2) * 255), 0), 255);
}

//Ramdom
struct  Random{
	std::mt19937 engine;
	std::uniform_real_distribution<double> dist;
	Random() {};
	Random(int seed) {
		engine.seed(seed);
		dist.reset();
	}
	// [0,1]�͈̔͂Ń����_���𐶐�����
	double next() { return dist(engine); }
};


// Ray
struct Ray {
	V o;	// ���_
	V d;	// ����
};

struct Sphere;
// Hit
struct Hit {
	double t;	// Ray�̌��_������������_�܂ł̋���
	V p;	// ���������_
	V n;	// p�̖@��
	const Sphere* sphere;	// �����������i�|�C���^�j
};

//Sphere
struct Sphere {
	V p;		// ���S�̈ʒu
	double r;	// ���a
	V R;		// reflectance �F ���˗�
	V Le;		// �Ɠx illuminance

	// ��������
	// ���������Ȃ��Hit�^�̒l��Ԃ��A�������Ă��Ȃ��ꍇ��std::nullptr��Ԃ�
	// Ray�̑��ݔ͈͂�Ray�̌��_����̋���[tmin,tmax]�Ŏw��
	std::optional<Hit> intersect(const Ray& ray, double tmin, double tmax) const {
		const V op = p - ray.o;

		// ������������
		// 1. ���ʏ�̓_x: |x-p|=r
		// 2. Ray��̓_x: x=o+t*d
		// 2��1�ɑ��
		const double b = dot(op, ray.d);

		// ���ʎ�
		const double det = b * b - dot(op, op) + r * r;
		if (det < 0) { return {}; }
		
		const double t1 = b - sqrt(det);
		if (tmin < t1 && t1 < tmax) { return Hit{ t1, {}, {}, this }; }
		const double t2 = b + sqrt(det);
		if (tmin < t2 && t2 < tmax) { return Hit{ t2, {}, {}, this }; }
		return {};
	}
};
//Scene
struct Scene {
	// �V�[�����ɑ��݂��鋅
	std::vector<Sphere> spheres{
		// 1
		//{ V(-.5, 0, 0), 1, V(1,0,0) },
		//{ V( .5, 0, 0), 1, V(0,1,0) },

		// Cornell Box
		{ V(1e5 + 1, 40.8, 81.6),	1e5, V(.75,.25,.25) },
		{ V(-1e5 + 99, 40.8, 81.6), 1e5, V(.25,.25,.75) },
		{ V(50, 40.8, 1e5),			1e5, V(.75) },
		{ V(50, 1e5, 81.6),			1e5, V(.75) },
		{ V(50, -1e5 + 81.6, 81.6), 1e5, V(.75) },
		{ V(27, 16.5, 47),			16.5, V(.999) },
		{ V(73, 16.5, 78),			16.5, V(.999) },
		{ V(50, 681.6-.27, 81.6),	600, V(), V(12) },
	};

	// ���������Ȃ��Hit�^�̒l��Ԃ��A�������Ă��Ȃ��ꍇ��std::nullptr��Ԃ�
	// Ray�̑��ݔ͈͂�Ray�̌��_����̋���[tmin,tmax]�Ŏw��
	std::optional<Hit> intersect(const Ray& ray, double tmin, double tmax) const {
		
		std::optional<Hit> minh;

		// ���ꂼ��̋��ɂ�������intersect���ĂсAt���ŏ��̂��̂�I������
		for (const auto& sphere : spheres) {
			// ���܂ł̋������擾
			const auto h = sphere.intersect(ray, tmin, tmax);

			// �������Ă��Ȃ��ꍇ�̓X�L�b�v
			if (!h) { continue; }

			// �ł������܂ł̋������Z�����̂��X�V
			minh = h;

			// �ő勗�����X�V
			tmax = minh->t;
		}

		// �@���̌v�Z
		if (minh) {
			const auto* s = minh->sphere;
			minh->p = ray.o + (ray.d * V(minh->t));
			minh->n = (minh->p - s->p) / V(s->r);
		}
		return minh;
	}

};


int main(int argc, char* argv[])
{
	// �J�n����
	auto start = clock();

	int args = 1000;
	int argd = 10;
	int argw = 1200;
	int argh = 800;

	for (int i = 0; i < argc; i++) {

		if (i == 0) { continue; }

		if (i == 1) { args = atoi(argv[i]); }
		if (i == 2) { argd = atoi(argv[i]); }
		if (i == 3) { argw = atoi(argv[i]); }
		if (i == 4) { argh = atoi(argv[i]); }
	}
	std::cout << "args:" << args << std::endl;
	std::cout << "argd:" << argd << std::endl;
	std::cout << "argw:" << argw << std::endl;
	std::cout << "argh:" << argh << std::endl;

	// file name stream
	std::stringstream fns;
	fns << "result_"
		<< args << "_"
		<< argd << "_"
		<< argw << "_"
		<< argh ;

	// Image Size
	const int w = argw;
	const int h = argh;

	// Sample per pixel
	int spp = args;

	// depth
	int depthCount = argd;

	// Camera patameters
	// 1
	//const V eye(5, 5, 5);		// �J�����ʒu
	//const V center(0, 0, 0);	// �J���������_
	//const V up(0, 1, 0);		// ������̒�`
	//const double fov = 30 * M_PI/180;		// field of view

	// Cornell Box
	const V eye(50, 52, 295.6);		// �J�����ʒu
	const V center = eye + V(0, -0.042612, -1);	// �J���������_
	const V up(0, 1, 0);		// ������̒�`
	const double fov = 30 * M_PI / 180;		// field of view

	const double aspect = (double)w / (double)h;// �A�X�y�N�g��
	// Basis vectors for camera coordinates
	const auto wE = normarize(eye - center);	// �J���������̔���
	const auto uE = normarize(cross(up, wE));
	const auto vE = cross(wE, uE);

	Scene scene;
	std::vector<V> I(w*h);

	// Sample per pixel
	//int spp = 1;

	// OpenMP���g���ĕ���
#pragma omp parallel for schedule(dynamic,1)
	for (int i = 0; i < w*h; i++) {
		
		// thread���ƂɃ����_�������@
		thread_local Random rng(42 + omp_get_thread_num());

		// �s�N�Z�����Ƃɕ����̃T���v�����v�Z����
		// �ŏI�I�ȋP�x�̓T���v���̕��ςŋ��߂�
		for (int j = 0; j < spp; j++) {
			// Ray
			const int x = i % w;	// x������
			const int y = h - i / w;	// y������(�㉺���])
			Ray ray;
			ray.o = eye;
			ray.d = [&]() {
				const double tf = std::tan(fov*.5);
				const double rpx = 2.*(x+rng.next()) / w - 1;	// �s�N�Z�����̃����_���Ȉʒu�����߂�
				const double rpy = 2.*(y+rng.next()) / h - 1;
				const V w = normarize(
					V(aspect*tf*rpx, tf*rpy, -1));	// �A�X�y�N�g��Efov�ɍ��킹���J�������W�i�X�N���[�����W�j
				return uE * w.x + vE * w.y + wE * w.z;	// ���[���h���W�ɕϊ�
			}();

			// L �P�x
			// th:througout ���̌o�H��ʂ����P�x�̕ω����L�^���邽�߂̕ϐ�
			V L(0), th(1);

			// ���̔��˂����X�ƌv�Z���Ă���
			// depth: ���ˉ�
			for (int depth = 0; depth < depthCount; depth++) {

				// Intersection
				// ���˂����Ƃ������ɓ�����Ȃ��悤�ɁAtmin�ɂ͏������l������Ă���
				const auto h = scene.intersect(ray, 1e-4, 1e+10);

				// ����������Ȃ���Δ�����
				if (!h) { break; }

				// Add contribution
				L = L + th * h->sphere->Le;

				// Update next direction
				// ���C�̌��_���X�V
				ray.o = h->p;
				// ���C�̕���
				ray.d = [&]() {
					// �Q�����C�ȍ~�̐���
					// 1. �ڋ�Ԃ̊��x�N�g�������߂�
					// 2. 1�̋�ԂŎ��̕����������_���ɐ���
					// 3. 2�̕��������[���h���W�n�ɕϊ�

					// Sample direction in local coodinates
					// �����͖@���Ɣ��˕�����cos�ɏ]�����z���琶������
					const auto n = dot(h->n, -ray.d) > 0 ? h->n : -h->n;
					
					// ����Ԋ��x�N�g�������߂�
					const auto&[u, v] = tangentSpace(n);

					const auto d = [&]() {
						const double r = sqrt(rng.next());
						const double t = 2 * M_PI * rng.next();
						const double x = r * cos(t);
						const double y = r * sin(t);
						return V(x, y, std::sqrt(std::max(.0, 1 - x * x - y * y)));
					}();
					// Convert to world coordinates
					return u * d.x + v * d.y + n * d.z;
				}();

				// Update throughput
				th = th * h->sphere->R;
				if (std::max({ th.x, th.y, th.z }) == 0) {
					break;
				}
			}

			I[i] = I[i] + L / spp;
		}

		//const auto c = h->sphere->R * dot(h->n, -ray.d);
		//I[i] = V(
			//std::abs(c.x),
			//std::abs(c.y),
			//std::abs(c.z));
	}

	//std::ofstream ofs("result.ppm");
	std::ofstream ofs(fns.str() += ".ppm");
	ofs << "P3\n" << w << " " << h << "\n255\n";
	for (const auto& i : I) {
		ofs << tonemap(i.x) << " "
			<< tonemap(i.y) << " "
			<< tonemap(i.z) << "\n";
	}

	// �I������
	auto end = clock();

	std::cout << "Time:" << end - start << std::endl;

	std::ofstream t_ofs(fns.str() += ".txt");
	t_ofs << end - start;

	return 0;
}