#define _USE_MATH_DEFINES
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
	// ガンマ補正をかけてから 0 - 255 でクランプする
	v = std::abs(v);
	return std::min(std::max(int(std::pow(v, 1 / 2.2) * 255), 0), 255);
}

// Ray
struct Ray {
	V o;	// 原点
	V d;	// 方向
};

struct Sphere;
// Hit
struct Hit {
	double t;	// Rayの原点から交差した点までの距離
	V p;	// 交差した点
	V n;	// pの法線
	const Sphere* sphere;	// あたった球（ポインタ）
};

//Sphere
struct Sphere {
	V p;		// 中心の位置
	double r;	// 半径

	// 交差判定
	// 交差したならばHit型の値を返す、交差していない場合はstd::nullptrを返す
	// Rayの存在範囲をRayの原点からの距離[tmin,tmax]で指定
	std::optional<Hit> intersect(const Ray& ray, double tmin, double tmax) const {
		const V op = p - ray.o;

		// 方程式を解く
		// 1. 球面上の点x: |x-p|=r
		// 2. Ray上の点x: x=o+t*d
		// 2を1に代入
		const double b = dot(op, ray.d);

		// 判別式
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
	// シーン中に存在する球
	std::vector<Sphere> spheres{
		// 1
		//{ V(-.5, 0, 0), 1 },
		//{ V( .5, 0, 0), 1 },

		// Cornell Box
		{ V(1e5 + 1, 40.8, 81.6), 1e5 },
		{ V(-1e5 + 99, 40.8, 81.6), 1e5 },
		{ V(50, 40.8, 1e5), 1e5 },
		{ V(50, 1e5, 81.6), 1e5 },
		{ V(50, -1e5 + 81.6, 81.6), 1e5 },
		{ V(27, 16.5, 47), 16.5 },
		{ V(73, 16.5, 78), 16.5 },
		{ V(50, 681.6-.27, 81.6), 600 },
	};

	// 交差したならばHit型の値を返す、交差していない場合はstd::nullptrを返す
	// Rayの存在範囲をRayの原点からの距離[tmin,tmax]で指定
	std::optional<Hit> intersect(const Ray& ray, double tmin, double tmax) const {
		
		std::optional<Hit> minh;

		// それぞれの球にたいしてintersectを呼び、tが最小のものを選択する
		for (const auto& sphere : spheres) {
			// 球までの距離を取得
			const auto h = sphere.intersect(ray, tmin, tmax);

			// 交差していない場合はスキップ
			if (!h) { continue; }

			// 最も交差までの距離が短いものを更新
			minh = h;

			// 最大距離を更新
			tmax = minh->t;
		}

		// 法線の計算
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
	// Image Size
	const int w = 1200;
	const int h = 800;

	// Camera patameters
	// 1
	//const V eye(5, 5, 5);		// カメラ位置
	//const V center(0, 0, 0);	// カメラ注視点
	//const V up(0, 1, 0);		// 上方向の定義
	//const double fov = 30 * M_PI/180;		// field of view

	// Cornell Box
	const V eye(50, 52, 295.6);		// カメラ位置
	const V center = eye + V(0, -0.042612, -1);	// カメラ注視点
	const V up(0, 1, 0);		// 上方向の定義
	const double fov = 30 * M_PI / 180;		// field of view

	const double aspect = (double)w / (double)h;// アスペクト比
	// Basis vectors for camera coordinates
	const auto wE = normarize(eye - center);	// カメラ向きの反対
	const auto uE = normarize(cross(up, wE));
	const auto vE = cross(wE, uE);

	Scene scene;
	std::ofstream ofs("result.ppm");
	ofs << "P3\n" << w << " " << h << "\n255\n";
	for (int i = 0; i < w*h; i++) {

		// Ray
		const int x = i % w;	// x軸成分
		const int y = h - i / w;	// y軸成分(上下反転)
		Ray ray;
		//ray.o = V(2.*double(x) / w - 1, 2.*double(y) / h - 1, 5);
		//ray.d = V(0, 0, -1);
		ray.o = eye;
		ray.d = [&]() {
			const double tf = std::tan(fov*.5);
			const double rpx = 2.*x / w - 1;
			const double rpy = 2.*y / h - 1;
			const V w = normarize(
				V(aspect*tf*rpx, tf*rpy, -1));	// アスペクト比・fovに合わせたカメラ座標（スクリーン座標）
			return uE * w.x + vE * w.y + wE * w.z;	// ワールド座標に変換
		}();

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