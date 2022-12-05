package main

import (
	"fmt"
	"image"
	"image/color"
	"image/png"
	"math"
	"math/rand"
	"os"
	"sort"
	"sync"

	"gonum.org/v1/gonum/floats"
	"gonum.org/v1/gonum/stat/distuv"
)

type light_ray struct {
	angle      float64 //rad
	pos        cart_point
	env_ior    float64
	lambda     float64 //nm
	brightness float64
}

type cart_point struct {
	x float64
	y float64
}

type obstacle struct {
	points []cart_point
	ior    func(float64) float64
}

type vector struct {
	A cart_point
	B cart_point
}

func planck(wav, T float64) float64 {
	v := c / wav
	a := 2.0 * h * math.Pow(v, 3) / math.Pow(c, 2)
	b := 1.0 / (h*v/(math.E*k*T) - 1.0)
	intensity := a * b * 100000
	return intensity
}

func generateBlackbodyRadiationSpectrum(resolution, T float64) ([]float64, []float64) {
	population := []float64{}
	weights := []float64{}
	x := 0.
	for x*resolution+visible_light_range[0] < visible_light_range[1] {
		currentWaveLength := x*resolution + visible_light_range[0]
		population = append(population, currentWaveLength)
		weights = append(weights, planck(currentWaveLength*1e-9, T))
		x += 1.
	}
	return population, weights
}

func wavelength_to_rgb(wavelength float64) (float64, float64, float64) {
	gamma := 0.8
	R := 0.0
	G := 0.0
	B := 0.0
	if (wavelength >= 380.) && (wavelength <= 440.) {
		attenuation := 0.3 + 0.7*(wavelength-380)/(440-380)
		R = math.Pow(((-(wavelength - 440) / (440 - 380)) * attenuation), gamma)
		G = 0.0
		B = math.Pow((1.0 * attenuation), gamma)
	} else if wavelength >= 440 && wavelength <= 490 {
		R = 0.0
		G = math.Pow(((wavelength - 440) / (490 - 440)), gamma)
		B = 1.0
	} else if wavelength >= 490 && wavelength <= 510 {
		R = 0.0
		G = 1.0
		B = math.Pow((-(wavelength - 510) / (510 - 490)), gamma)
	} else if wavelength >= 510 && wavelength <= 580 {
		R = math.Pow(((wavelength - 510) / (580 - 510)), gamma)
		G = 1.0
		B = 0.0
	} else if wavelength >= 580 && wavelength <= 645 {
		R = 1.0
		G = math.Pow((-(wavelength - 645) / (645 - 580)), gamma)
		B = 0.0
	} else if wavelength >= 645 && wavelength <= 750 {
		attenuation := 0.3 + 0.7*(750-wavelength)/(750-645)
		R = math.Pow((1.0 * attenuation), gamma)
		G = 0.0
		B = 0.0
	} else {
		R = 0.0
		G = 0.0
		B = 0.0
	}
	return R, G, B
}

func determine_n_for_bk_seven_glass(wav float64) float64 {
	wavum := wav * 1e-3
	B1 := 1.03961212
	B2 := 0.231792344
	B3 := 1.01046945
	C1 := 6.00069867e-3
	C2 := 2.00179144e-2
	C3 := 1.03560653e2
	n_squared := 1 + (B1*math.Pow(wavum, 2))/(math.Pow(wavum, 2)-C1) + (B2*math.Pow(wavum, 2))/(math.Pow(wavum, 2)-C2) + (B3*math.Pow(wavum, 2))/(math.Pow(wavum, 2)-C3)
	return math.Sqrt(n_squared)
}

func ccw(A, B, C cart_point) bool {
	return ((C.y-A.y)*(B.x-A.x) > (B.y-A.y)*(C.x-A.x))
}

func intersect_check(vecA, vecB vector) bool {
	A := vecA.A
	B := vecA.B
	C := vecB.A
	D := vecB.B
	return (ccw(A, C, D) != ccw(B, C, D) && ccw(A, B, C) != ccw(A, B, D))
}

func get_vec_normal_abs_angle(vec vector) float64 {
	return math.Atan2((vec.A.y-vec.B.y), (vec.B.x-vec.A.x)) + math.Pi/2.0 // 90° = math.Pi/2.0
}

func exiting_obstacle_check(obstacle_ior, simulated_light_ray_env_ior float64) bool {
	return (air_ior != simulated_light_ray_env_ior)
}

func refraction(simulated_light_ray_vector vector, simulated_light_ray light_ray) light_ray {
	for _, current_obstacle := range obstacles {
		for i, _ := range current_obstacle.points {
			side := vector{current_obstacle.points[i], current_obstacle.points[(i+1)%len(current_obstacle.points)]}
			if intersect_check(side, simulated_light_ray_vector) {
				vec_normal_abs_angle := get_vec_normal_abs_angle(side)
				is_exiting_obstacle := exiting_obstacle_check(current_obstacle.ior(simulated_light_ray.lambda), simulated_light_ray.env_ior)
				new_ior := current_obstacle.ior(simulated_light_ray.lambda)
				if is_exiting_obstacle {
					new_ior = air_ior
				}
				incident_angle := simulated_light_ray.angle - vec_normal_abs_angle
				hold_val := incident_angle - math.Asin(math.Sin(incident_angle))
				// fmt.Println("same val:")
				// fmt.Println(incident_angle)
				// fmt.Println(math.Asin(math.Sin(incident_angle)))
				// fmt.Println("corrected:")
				// fmt.Println(math.Asin(math.Sin(incident_angle)) + hold_val)
				// fmt.Println("int:")
				// fmt.Println(math.Sin(vec_normal_abs_angle - simulated_light_ray.angle))

				simulated_light_ray.angle = hold_val + math.Asin(simulated_light_ray.env_ior*math.Sin(incident_angle)/new_ior) + vec_normal_abs_angle
				simulated_light_ray.env_ior = new_ior
			}
		}
	}
	return simulated_light_ray
}

func update_light_ray(simulated_light_ray light_ray) light_ray {
	simulated_light_ray_prev_pos := simulated_light_ray.pos
	simulated_light_ray.pos.x += light_speed * time_step * math.Cos(simulated_light_ray.angle)
	simulated_light_ray.pos.y += -light_speed * time_step * math.Sin(simulated_light_ray.angle)
	simulated_light_ray_vector := vector{simulated_light_ray_prev_pos, simulated_light_ray.pos}
	simulated_light_ray = refraction(simulated_light_ray_vector, simulated_light_ray)
	return simulated_light_ray
}

func weighted_rand(population, weights []float64) float64 {
	cdf := make([]float64, len(weights))
	floats.CumSum(cdf, weights)
	// multiply the sample with the largest CDF value; easier than normalizing to [0,1)
	val := distuv.UnitUniform.Rand() * cdf[len(cdf)-1]
	// Search returns the smallest index i such that cdf[i] > val
	return population[sort.Search(len(cdf), func(i int) bool { return cdf[i] > val })]
}

func simulate_light_ray(ray_count_per_thread int, emission_population, emission_weights []float64, imageArray [][][3]float64, wg *sync.WaitGroup) {
	defer wg.Done()
	for r := 0; r < ray_count_per_thread; r++ {
		if r%printEvery == 0 {
			fmt.Println(r*100/ray_count_per_thread, "%")
		}
		light_ray_starting_angle := rand.Float64()*(maximum_angle/180.*math.Pi-minimum_angle/180.*math.Pi) + minimum_angle/180.*math.Pi
		light_ray_starting_point := cart_point{rand.Float64()*(maximum_source_point.x-minimum_source_point.x) - maximum_source_point.x, rand.Float64()*(maximum_source_point.y-minimum_source_point.y) - maximum_source_point.y}
		light_ray_wavelength := weighted_rand(emission_population, emission_weights)
		light_ray_r, light_ray_g, light_ray_b := wavelength_to_rgb(light_ray_wavelength)
		simulated_light_ray := light_ray{light_ray_starting_angle, cart_point{light_ray_starting_point.x, light_ray_starting_point.y}, air_ior, light_ray_wavelength, ray_brightness}
		for i := 0; i < ray_length; i++ {
			pixelX := int(math.Round((simulated_light_ray.pos.x * scale))) + width/2
			pixelY := int(math.Round((-simulated_light_ray.pos.y * scale))) + height/2
			if pixelX < 0 || pixelX > width-1 {
				break
			}
			if pixelY < 0 || pixelY > height-1 {
				break
			}
			imageArray[pixelX][pixelY][0] += simulated_light_ray.brightness * light_ray_r
			imageArray[pixelX][pixelY][1] += simulated_light_ray.brightness * light_ray_g
			imageArray[pixelX][pixelY][2] += simulated_light_ray.brightness * light_ray_b
			simulated_light_ray = update_light_ray(simulated_light_ray)
		}
	}
}

func draw_ellipse(a, b float64, points int, center_point cart_point, ior func(float64) float64) obstacle {
	ellipse := obstacle{[]cart_point{}, ior}
	for pointN := 0; pointN < points; pointN++ {
		theta := math.Pi * 2.0 * float64(pointN) / float64(points)
		calcX := center_point.x + a*math.Cos(theta)
		calcY := center_point.y + b*math.Sin(theta)
		ellipse.points = append(ellipse.points, cart_point{calcX, calcY})
	}
	return ellipse
}

// Simulation settings
var light_speed = 1.0
var time_step = 0.002
var air_ior = 1.0
var ray_length = 1000000
var ray_count = 100000
var minimum_angle = 0.   // deg
var maximum_angle = 360. // deg
var minimum_source_point = cart_point{0., 0.}
var maximum_source_point = cart_point{0., 0.}
var threads = 8
var ray_brightness = .1
var obstacles = []obstacle{
	obstacle{[]cart_point{cart_point{20.0, -20.0}, cart_point{40.0, -20.0}, cart_point{40.0, 20.0}, cart_point{20.0, 20.0}}, determine_n_for_bk_seven_glass},
	obstacle{[]cart_point{cart_point{-30.0, -10.0}, cart_point{-10.0, -10.0}, cart_point{-20.0, 10.0}}, determine_n_for_bk_seven_glass},
	//obstacle{[]cart_point{cart_point{-40.0, -20.0}, cart_point{-20.0, -20.0}, cart_point{-20.0, 20.0}, cart_point{-40.0, 20.0}}, determine_n_for_bk_seven_glass},
	draw_ellipse(10.0, 40.0, 100, cart_point{-65., 0.}, determine_n_for_bk_seven_glass),
}

// Display settings
var width = 2000
var height = 200
var scale = 40.0

// Color
var visible_light_range = [2]float64{380., 750.}
var light_temperature = 6900. // K

// Constants
var h = 6.626e-34
var c = 3.0e+8
var k = 1.38e-23

// Program settings
var printEvery = 125

func main() {
	// Generate lightsource emission spectrum
	emission_population, emission_weights := generateBlackbodyRadiationSpectrum(0.5, light_temperature)
	// Simulation
	imageArray := make([][][3]float64, width)
	for i := range imageArray {
		imageArray[i] = make([][3]float64, height)
	}
	wg := &sync.WaitGroup{}
	ray_count_per_thread := ray_count / threads
	for a := 0; a < threads; a++ {
		wg.Add(1)
		go simulate_light_ray(ray_count_per_thread, emission_population, emission_weights, imageArray, wg)
	}
	wg.Wait()
	// Display
	upLeft := image.Point{0, 0}
	lowRight := image.Point{width, height}

	img := image.NewRGBA(image.Rectangle{upLeft, lowRight})
	for pixelX := 0; pixelX < width; pixelX++ {
		for pixelY := 0; pixelY < height; pixelY++ {
			img.Set(pixelX, pixelY, color.RGBA{uint8(math.Min(imageArray[pixelX][pixelY][0], 255)), uint8(math.Min(imageArray[pixelX][pixelY][1], 255)), uint8(math.Min(imageArray[pixelX][pixelY][2], 255)), 255})
		}
	}
	// Encode as PNG.
	f, _ := os.Create("image.png")
	png.Encode(f, img)
}
