#include <iostream>
#include <fstream>
#include <vector>
#include <optional>
#include <algorithm>

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

//Tone Map
int tonemap(double v) {
	// �K���}�␳�������Ă��� 0 - 255 �ŃN�����v����
	return std::min(std::max(int(std::pow(v, 1 / 2.2) * 255), 0), 255);
}

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
		
		const double t1 = b + sqrt(det);
		if (tmin < t1 && t1 < tmax) { return Hit{ t1, V(), V(), this }; }
		const double t2 = b - sqrt(det);
		if (tmin < t2 && t2 < tmax) { return Hit{ t2, V(), V(), this }; }
		return {};
	}
};
//Scene
struct Scene {
	// �V�[�����ɑ��݂��鋅
	std::vector<Sphere> spheres{
		{ V(), 1 }
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


int main()
{
	const int w = 1200;
	const int h = 800;
	Scene scene;
	std::ofstream ofs("result.ppm");
	ofs << "P3\n" << w << " " << h << "\n255\n";
	for (int i = 0; i < w*h; i++) {

		// Ray
		const int x = i % w;	// x������
		const int y = i / w;	// y������
		Ray ray;
		ray.o = V(2.*double(x) / w - 1, 2.*double(y) / h - 1, 5);
		ray.d = V(0, 0, -1);

		const auto h = scene.intersect(ray, 0, 1e+10);
		if (h) {
			const auto n = h->n;

			ofs << tonemap(n.x) << " "
				<< tonemap(n.y) << " "
				<< tonemap(n.z) << "\n";
		}
		else {
			ofs << "0 0 0\n";
		}
	}
	return 0;
}