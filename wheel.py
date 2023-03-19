from manim import *
from manim_cad_drawing_utils import *


class Path_Offset_Mobject_Warped(Path_Offset_Mobject):
    def __init__(self,
                 target_mobject,
                 ofs_func,
                 ofs_func_kwargs={},
                 warp_func=lambda t: t,
                 num_of_samples=100,
                 discontinuities=[],
                 **kwargs):
        self.warp_func = warp_func
        super().__init__(target_mobject,
                         ofs_func,
                         ofs_func_kwargs=ofs_func_kwargs,
                         num_of_samples=num_of_samples,
                         discontinuities=discontinuities,
                         **kwargs)


    def generate_normal_vectors(self):
        s_range = self.warp_func(self.t_range) * self.PM.get_path_length()
        # generate normal vectors from tangent angles and turning them 90°
        # the angles can be interpolated with bezier, unit normal vectors would not remain 'unit' under interpolation
        angles = self.generate_bezier_points(self.PM.get_tangent_angle, s_range)
        out = []
        for angle in angles:
            out.append([np.array([-np.sin(a), np.cos(a), 0]) for a in angle])
        return out
    def generate_ref_curve(self):
        self.ref_curve = VMobject()
        bez_point = self.generate_bezier_points(self.PM.point_from_proportion, self.warp_func(self.t_range))
        for point in bez_point:
            self.ref_curve.append_points(point)
        self.ref_curve_path = Path_mapper(self.ref_curve)




class Wheel_test_2(MovingCameraScene):
    def construct(self):
        # mob1 = Round_Corners(Square().scale(5), radius=1).shift(DOWN * 2)
        R = 2.8
        R_gnd=ValueTracker(2.81)
        bump_cnt = 16
        def wheel_shape_generator():
            wheel_circle=Circle(R,num_components=20).rotate(-PI*0.25)
            wheel_mask = Rectangle(width=R*2, height=2*R).shift((R+R_gnd.get_value())*DOWN)
            mob0 = Difference(wheel_circle,wheel_mask)
            mob1 = Path_Offset_Mobject(Round_Corners(mob0.copy(),1,arc_components=20),lambda t: 0,num_of_samples=200)
            mob1.set_stroke(opacity=0)
            mob1.set_fill(opacity=0)
            mob1.points = np.roll(mob1.points,4*23,0)
            return mob1
        mob1 = wheel_shape_generator()
        # mob1.add_updater(lambda mob: mob.match_points(wheel_shape_generator()))
        drift = ValueTracker(0.0)
        phase = ValueTracker(0)
        warp_phase = ValueTracker(0.0)
        warp_expansion = ValueTracker(0.0)
        rot_vt = ValueTracker(PI/2)
        bump_height = 0.1

        def ofs_func(t):
            if ((t-phase.get_value()%1)*bump_cnt)%1>0.5:
                return 0
            else:
                return bump_height
        def warp_func(t):
            points = np.array([0, drift.get_value(), 0])
            t_warp = (np.clip(t, 0, 1) - warp_phase.get_value()) % 1
            return bezier(points)(t_warp)
        def shift_func(t):
            points = np.array([0,drift.get_value(),0])
            val = t + warp_func(t) - bezier(points)((0-warp_phase.get_value())%1)
            return val

        def ofs_generator():
            discont_points = (np.array([z * 0.5/bump_cnt + phase.get_value()for z in range(int( bump_cnt*2))]))
            return Path_Offset_Mobject_Warped(mob1,
                                              ofs_func,
                                              warp_func=shift_func,
                                              num_of_samples=bump_cnt,
                                              discontinuities=discont_points,
                                              fill_opacity=0,
                                              fill_color=RED,
                                              stroke_opacity=1,
                                              stroke_color=WHITE)


        Wheel = ofs_generator()
        Wheel.add_updater(lambda mob: mob.match_points(ofs_generator()))
        def wheel_updater(mob:Path_Offset_Mobject_Warped):
            discont_points = (np.array([z * 0.5 / bump_cnt + phase.get_value() for z in range(int(bump_cnt * 2))]))
            # mob.ofs_func=ofs_func
            Wheel.update_t_range(discontinuities=discont_points, num_of_samples=bump_cnt)
            Wheel.points=mob.generate_offset_paths()
        # Wheel.add_updater(wheel_updater)
        # mob0.set_color(RED)

        R_hub = R*0.4
        Hub=VGroup(Circle(radius=R_hub))
        n_bolt=8
        for k in range(n_bolt):
            Hub.add(Circle(radius=R_hub/10).shift(rotate_vector(RIGHT*R_hub*0.66,TAU*k/n_bolt)))
        Hub.add(Circle(radius=R_hub*0.3))
        Hub.add(Circle(radius=R_hub*0.9))
        Hub.set_color(WHITE)
        Hub_ref = Hub.copy()
        Hub.add_updater(lambda mob: mob.match_points(Hub_ref).rotate((phase.get_value()+1.2*warp_func(0.5))*TAU))

        def spline_generator(t_start,t_end):
            v0 = normalize(Wheel.points[0, :])
            R_start = R_hub + 0.1
            R_end = (R - bump_height) - 0.1
            tau = t_start + 1.2 * warp_func(0.5) % 1
            start_point = rotate_vector(v0, tau * TAU) * R_start
            end_point = rotate_vector(v0, t_end * TAU) * R_end
            mob = VMobject()
            mob.set_points_as_corners([start_point, end_point])
            mob.points[1, :] = mob.points[0, :] * (1 + (R_end - R_start) / R_start * 0.5)
            mob.points[2, :] = (mob.points[3, :] + mob.points[1, :]) / 2
            return mob

        def spline_grp_generator():
            start_ts = (np.array([z * 0.5 / bump_cnt + phase.get_value() for z in range(int(bump_cnt * 2))]))%1
            end_ts = start_ts + warp_func(start_ts)
            spline_grp = VGroup()
            for k in range(len(start_ts)):
                spline_grp.add(spline_generator(start_ts[k],end_ts[k]))
            return spline_grp

        Spline_grp = spline_grp_generator()
        Spline_grp.set_stroke(width=2, color=RED)
        Spline_grp.add_updater(lambda mob: mob.match_points(spline_grp_generator()))

        BaseLine = DashedVMobject(Line(LEFT*60,RIGHT*60,stroke_width=10),300,0.8).shift(DOWN*(R_gnd.get_value()+7/100)+LEFT*(20))

        # phase.set_value(0.01)
        # drift.set_value(0.15)
        # Wheel.update()
        # dbg = Bezier_Handlebars(Wheel)
        # dbg['dots'][1].set_color(RED)
        # dbg.add_updater(lambda mob: mob.match_points(Bezier_Handlebars(Wheel)))

        self.add(mob1)
        self.play(Create(Wheel),
                  Create(Hub),
                  Create(BaseLine))
        self.wait(3)
        roll=0.1
        self.play(phase.animate.set_value(roll),
                  BaseLine.animate.shift(RIGHT*(roll-2*drift.get_value())*(Wheel.PM.get_path_length())),
                  self.camera.frame.animate.shift(RIGHT*(roll-2*drift.get_value())*(Wheel.PM.get_path_length()))
                  )
        self.play(phase.animate.set_value(0),
                  BaseLine.animate.shift(RIGHT*(-roll)*(Wheel.PM.get_path_length())),
                  self.camera.frame.animate.shift(RIGHT*(-roll)*(Wheel.PM.get_path_length()))
                  )
        self.wait()
        R_gnd.set_value(2.65)
        gnd_shift = R-R_gnd.get_value()
        mob1=wheel_shape_generator()
        Wheel.suspend_updating()
        self.play(Transform(Wheel,ofs_generator()),
                  BaseLine.animate.shift(UP*(gnd_shift)),
                  self.camera.frame.animate.shift(UP*(gnd_shift)))
        Wheel.resume_updating()
        self.wait(2)
        roll=-0.1
        self.play(phase.animate.set_value(roll),
                  BaseLine.animate.shift(RIGHT*(roll-2*drift.get_value())*(Wheel.PM.get_path_length())),
                  self.camera.frame.animate.shift(RIGHT*(roll-2*drift.get_value())*(Wheel.PM.get_path_length()))
                  )
        self.play(phase.animate.set_value(0),
                  BaseLine.animate.shift(RIGHT*(-roll)*(Wheel.PM.get_path_length())),
                  self.camera.frame.animate.shift(RIGHT*(-roll)*(Wheel.PM.get_path_length()))
                  )
        self.wait()
        R_gnd.set_value(R+0.01)
        mob1=wheel_shape_generator()
        Wheel.suspend_updating()
        self.play(Transform(Wheel,ofs_generator()),
                  BaseLine.animate.shift(DOWN*(gnd_shift)),
                  self.camera.frame.animate.shift(DOWN*(gnd_shift)))
        Wheel.resume_updating()
        self.wait()
        self.play(Create(Spline_grp))
        self.wait()
        self.play(drift.animate.set_value(0.15), run_time=4)
        self.wait()

        cam_shift_vt = ValueTracker()
        cam_shift_vt2 = ValueTracker()
        self.camera.frame.add_updater(lambda mob: mob.move_to(RIGHT*(cam_shift_vt.get_value()-cam_shift_vt2.get_value())))
        self.play(AnimationGroup(AnimationGroup(phase.animate.set_value(1),
                  BaseLine.animate.shift(RIGHT*(1/1-2*drift.get_value())*(Wheel.PM.get_path_length()))),
                                 cam_shift_vt.animate.set_value(8),
                                 cam_shift_vt2.animate.set_value(8),
                                 lag_ratio=1/16),
                  run_time=16)
        self.camera.frame.clear_updaters()
        self.wait()

        if 1:
            # self.play(drift.animate.set_value(0), run_time=4)

            Hub_empty = Circle(R_hub)
            Ref_spline = spline_generator(0,0)
            Ref_spline.set_stroke(color=GREEN)

            Spline_vt = ValueTracker(0)
            Ref_spline.add_updater(
                lambda mob: mob.match_points(spline_generator(Spline_vt.get_value()%1,
                                                              Spline_vt.get_value()%1 + warp_func(Spline_vt.get_value()%1))))
            Spline_vt2 = ValueTracker(0)
            Ref_spline2=spline_generator(0,0)
            Ref_spline2.add_updater(
                lambda mob: mob.match_points(spline_generator(Spline_vt2.get_value()%1,
                                                              Spline_vt2.get_value()%1 + warp_func(Spline_vt2.get_value()%1))))
            self.play(FadeOut(Hub),
                      FadeIn(Hub_empty),
                      Spline_grp.animate.set_stroke(opacity=0.3))
            self.play(FadeIn(Ref_spline))
            self.wait()
            self.play(Spline_vt.animate.set_value(0.5), run_time=4)
            self.wait()
            self.play(Spline_vt.animate.set_value(1.1),run_time=4)

            A_dim_1 = Angle_Dimension_3point(Ref_spline.points[0,:],
                                             R_hub*DOWN,
                                             ORIGIN,
                                             offset=-R_hub/4,
                                             ext_line_offset=0,
                                             color=RED,
                                             text=MathTex('\phi',color=RED)
                                             )
            A_dim_1.add_updater(lambda mob: mob.match_points(
                Angle_Dimension_3point(Ref_spline.points[0,:],
                                       R_hub*DOWN,
                                       ORIGIN,
                                       offset=-R_hub/4,
                                       ext_line_offset=0,
                                       color=RED,
                                       text=MathTex('\phi',color=RED)
                                       )))
            A_dim_2 = Angle_Dimension_3point(Ref_spline.points[3,:],
                                             R* DOWN,
                                             ORIGIN,
                                             offset=0.6,
                                             ext_line_offset=-1.5,
                                             color=RED,
                                             text=MathTex('\gamma',color=RED)
                                             )
            A_dim_2.add_updater(lambda mob: mob.match_points(
                Angle_Dimension_3point(Ref_spline.points[3, :],
                                       R * DOWN,
                                       ORIGIN,
                                       offset=0.6,
                                       ext_line_offset=-0.15,
                                       color=RED,
                                       text=MathTex('\gamma', color=RED))
            ))
            self.play(FadeOut(BaseLine))
            self.play(Create(A_dim_1),Create(A_dim_2))
            self.wait()

            self.play(drift.animate.set_value(0.0), run_time=2)
            self.play(drift.animate.set_value(0.15), run_time=2)
            self.wait()

            ax = Axes([-2,4,1],
                      [-2,4,1],
                      x_length=5,
                      y_length=5,
                      x_axis_config={'include_numbers': True,
                                     "decimal_number_config": {
                                         "unit": r"\pi",
                                         "num_decimal_places": 0, }})
            ax.next_to(Wheel,buff=1)

            def plot_func(t):
                return t+2*2*(-warp_func(0.5)*1.2+warp_func((t/2)%1))
            disp_func = VGroup(*[ax.plot(plot_func,x_range=[(k-1)*2,k*2]) for k in range(3)])
            disp_func.add_updater(lambda mob: mob.match_points(
                VGroup(*[ax.plot(plot_func,x_range=[(k-1)*2,k*2]) for k in range(3)])
                                  ))
            ref_func = VGroup(*[ax.plot(lambda t: t,x_range=[(k-1)*2,k*2],color=RED) for k in range(3)])
            Phi_label = MathTex(r'\phi')
            Gamma_label = MathTex(r'\gamma')
            Gamma_label.next_to(ax.y_axis,UP+RIGHT)
            Phi_label.next_to(ax.x_axis, DOWN + RIGHT)

            self.play(self.camera.frame.animate.move_to(VGroup(ax,Wheel,Phi_label)))
            self.play(Create(VGroup(ax,disp_func[1],ref_func[1],Phi_label,Gamma_label)))


            self.wait()
            self.play(Create(VGroup(disp_func[0],disp_func[2],ref_func[0],ref_func[2])))
            self.wait()
            self.add(disp_func)
            # self.wait()



            eq1 = MathTex(r'\mathrm{d} s ','=','R',' \mathrm{d} \gamma').next_to(Wheel,DOWN)
            eq2 = MathTex(r'\mathrm{d} s ','=',r'R \frac{\mathrm{d} \gamma}{\mathrm{d} \phi} \mathrm{d} \phi ' ).next_to(Wheel, DOWN)
            eq3 = MathTex(r"\mathrm{d} s",'=',r"R f'_{(\phi)} \mathrm{d} \phi ").next_to(Wheel, DOWN)

            self.play(self.camera.frame.animate.move_to(eq1),
                      FadeOut(A_dim_1),
                      FadeOut(A_dim_2))

            Ref_spline2.set_color(GREEN)
            Spline_vt2.set_value(1)
            self.play(Create(Ref_spline2))
            self.play(Spline_vt.animate.set_value(1 - 1 / bump_cnt / 2))
            self.wait()

            self.play(Create(eq1))
            self.wait()
            self.play(TransformMatchingTex(eq1,eq2))
            self.wait()
            self.play(TransformMatchingTex(eq2, eq3))
            self.wait()
            self.play(FadeOut(eq3))
            self.play(self.camera.frame.animate.move_to(VGroup(ax, Wheel, Phi_label)))
            zerodot = Circle(arc_center=disp_func[1].points[0,:],radius=0.1,stroke_opacity=0,fill_opacity=1,fill_color=TEAL)
            self.play(AnimationGroup(GrowFromCenter(zerodot),Flash(zerodot,color=TEAL),lag_ratio=0.5))

            self.wait()

            diff_highlight  = ax.plot(plot_func,x_range=[-0.3,0.3],color=TEAL,stroke_width=6)
            self.play(Create(diff_highlight))
            self.wait()

            # self.play(drift.animate.set_value(0))
            # self.play(drift.animate.set_value(0.15))
            self.play(Spline_vt2.animate.set_value(1+1 / bump_cnt / 2),
                      Spline_vt.animate.set_value(1))
            self.wait()
            self.play(Spline_vt2.animate.set_value(1),
                      Spline_vt.animate.set_value(1-1/bump_cnt/2))
            self.play(Spline_vt2.animate.set_value(1 + 1 / bump_cnt / 2),
                      Spline_vt.animate.set_value(1))
            # #
            self.wait()

            self.play(FadeOut(Ref_spline),
                      FadeOut(Ref_spline2),
                      FadeOut(VGroup(ax,disp_func,ref_func,Phi_label,Gamma_label)),
                      FadeOut(VGroup(zerodot,diff_highlight)),
                      FadeIn(Hub),
                      FadeOut(Hub_empty))
            self.wait()
            self.play(FadeIn(BaseLine),
                      drift.animate.set_value(0),
                      Spline_grp.animate.set_stroke(opacity=1))

            Wheel_copy = VGroup(Wheel.copy(),Hub.copy(),Spline_grp.copy()).shift(DOWN*3*R)
            BaseLine_copy = BaseLine.copy().shift(DOWN*3*R)
            Wheel_copy.clear_updaters()
            Wheel_copy_ref = Wheel_copy.copy()
            Wheel_copy.add_updater(lambda mob: mob.match_points(Wheel_copy_ref.rotate(phase.get_value()*TAU,
                                                                                      about_point=Wheel_copy_ref[1][0].get_center())))
            BaseLine_copy.suspend_updating()
            self.add(Wheel_copy,BaseLine_copy)
            self.play(self.camera.frame.animate.scale(2.3).move_to(VGroup(Wheel,Wheel_copy)).shift(LEFT*R*PI*1.2))

            rot_amount = 1



            self.play(drift.animate.set_value(0.15))

            marker_1 = Circle(radius=R/10,
                              arc_center=Wheel.get_center()+R*DOWN,
                              fill_opacity=1,
                              fill_color=RED)
            marker_1_ref = marker_1.copy()
            mvt0=0.425
            marker_vt = ValueTracker(mvt0)
            marker_1.add_updater(lambda mob: mob.match_points(marker_1_ref).rotate(
                TAU*(marker_vt.get_value()+warp_func(marker_vt.get_value()%1)),
            about_point=ORIGIN))
            marker_1.update()
            marker_2 = Circle(radius=R / 10,
                              arc_center=Wheel_copy.get_center() + R * UP,
                              fill_opacity=1,
                              fill_color=RED)
            Wheel_copy_ref.add(marker_2.copy())
            Wheel_copy.add(marker_2)

            self.play(Create(marker_1),Create(marker_2))
            slip_shift_amount = (1 - 2 * drift.get_value()) * (Wheel.PM.get_path_length())
            norm_shift_amount = R * TAU
            self.play(phase.animate.set_value(2),
                      BaseLine.animate.shift(RIGHT * (slip_shift_amount)),
                      BaseLine_copy.animate.shift(RIGHT * (slip_shift_amount)),
                      self.camera.frame.animate.shift(RIGHT * (slip_shift_amount)),
                      Wheel_copy_ref.animate.shift(LEFT*(norm_shift_amount-slip_shift_amount)),
                      marker_vt.animate.set_value(1+mvt0),
                      run_time=16)
            self.wait()

            self.play(phase.animate.set_value(1),
                      BaseLine.animate.shift(LEFT * (slip_shift_amount)),
                      BaseLine_copy.animate.shift(LEFT * (slip_shift_amount)),
                      self.camera.frame.animate.shift(LEFT * (slip_shift_amount)),
                      Wheel_copy_ref.animate.shift(RIGHT * (norm_shift_amount - slip_shift_amount)),
                      marker_vt.animate.set_value(0 + mvt0),
                      run_time=2)
            self.wait()
            mvt1 = 1-0.425
            self.play(drift.animate.set_value(-0.15),marker_vt.animate.set_value(0 + mvt1))
            self.wait()
            slip_shift_amount = (1 - 2 * drift.get_value()) * (Wheel.PM.get_path_length())
            norm_shift_amount = R * TAU
            self.play(phase.animate.set_value(2),
                      BaseLine.animate.shift(RIGHT * (slip_shift_amount)),
                      BaseLine_copy.animate.shift(RIGHT * (slip_shift_amount)),
                      self.camera.frame.animate.shift(RIGHT * (slip_shift_amount)),
                      Wheel_copy_ref.animate.shift(LEFT*(norm_shift_amount-slip_shift_amount)),
                      marker_vt.animate.set_value(1+mvt1),
                      run_time=16)
            self.wait()

class Slip_graph(Scene):
    def construct(self):
        D=1
        C=2.3
        B=6
        E=1

        def magicformula(phi):
            return D*np.sin(C*np.arctan(B*phi-E*(B*phi-np.arctan(B*phi))))

        ax1 =Axes([-1,1,0.1],[-1.5,1.5,0.25])
        s_label = Text('s').next_to(ax1,RIGHT)
        F_label = Text('F').next_to(ax1, UP)
        MF_plot = ax1.plot(magicformula)
        MF_highlight = ax1.plot(magicformula,x_range=[-0.1,0.1],color=RED)


        eq_s = MathTex(r's=\frac{R\omega-v}{v}')
        self.play(Write(eq_s))
        self.wait()
        self.play(FadeOut(eq_s))
        self.play(Create(ax1))
        self.play(Create(s_label),
                  Create(F_label))
        self.play(Create(MF_plot))
        self.wait()
        self.play(Create(MF_highlight))
        self.wait()

if __name__=="__main__":
    with tempconfig({"quality": "medium_quality","save_last_frame": True, "disable_caching": True}):
        scene = Wheel_test_2()
        scene.render()