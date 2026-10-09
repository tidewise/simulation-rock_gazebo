# frozen_string_literal: true

require 'test/helpers'

describe 'rock_gazebo::ModelTask' do
    include Orocos::Test::Component
    include Helpers

    attr_reader :task

    describe 'creation by the plugin' do
        before do
            @task = gzserver 'model.world', '/gazebo::w::m'
        end

        it 'exports the model using a ModelTask' do
            assert_equal 'rock_gazebo::ModelTask', task.model.name
        end
    end

    describe 'the pose samples' do
        before do
            @task = gzserver 'model.world', '/gazebo::w::m'
        end

        it "exports the model's pose" do
            pose = configure_start_and_read_one_new_sample 'pose_samples'
            assert Eigen::Vector3.new(1, 2, 3).approx?(pose.position)
            assert Eigen::Quaternion.from_angle_axis(0.1, Eigen::Vector3.UnitZ)
                                    .approx?(pose.orientation)
        end

        it 'sets the model\'s cov_position from the component\'s '\
            'cov_position property' do
            cov = matrix3_rand
            task.cov_position = cov
            pose = configure_start_and_read_one_new_sample 'pose_samples'
            assert_matrix3_in_delta cov, pose.cov_position
        end

        it 'sets the model\'s cov_orientation from the component\'s '\
            'cov_orientation property' do
            cov = matrix3_rand
            task.cov_orientation = cov
            pose = configure_start_and_read_one_new_sample 'pose_samples'
            assert_matrix3_in_delta cov, pose.cov_orientation
        end

        it 'sets the model\'s cov_velocity from the component\'s '\
            'cov_velocity property' do
            cov = matrix3_rand
            task.cov_velocity = cov
            pose = configure_start_and_read_one_new_sample 'pose_samples'
            assert_matrix3_in_delta cov, pose.cov_velocity
        end
    end

    describe "the model's joints" do
        before do
            @task = gzserver 'joints_export.world', '/gazebo::w::m'
        end

        it 'exports the state of all the non-fixed joints' do
            reader = task.port('joints_samples').reader
            writer = task.port('joints_cmd').writer
            initialize_joint_positions(writer)

            joints = nil
            poll_until do
                if (sample = reader.read_new)
                    joints = sample
                    (-0.1..0.11).include?(joints.elements[0].position) &&
                        (0.19..0.31).include?(joints.elements[1].position) &&
                        (0.39..0.51).include?(joints.elements[2].position) &&
                        (0.59..0.71).include?(joints.elements[3].position)
                end
            end

            assert_equal %w[m::j_00 m::j_01 m::child_j_00 m::child_j_01],
                         joints.names

            assert_includes (-0.1..0.11), joints.elements[0].position
            assert_includes (0.19..0.31), joints.elements[1].position
            assert_includes (0.39..0.51), joints.elements[2].position
            assert_includes (0.59..0.71), joints.elements[3].position
        end

        it 'allows commanding the joints' do
            cmd = Types.base.samples.Joints.new(
                names: %w[m::j_00 m::j_01 m::child_j_00 m::child_j_01],
                elements: [Types.base.JointState.Effort(-1),
                            Types.base.JointState.Effort(1),
                            Types.base.JointState.Effort(-1),
                            Types.base.JointState.Effort(1)]
            )

            task.configure
            task.start
            reader = task.port('joints_samples').reader
            writer = task.port('joints_cmd').writer
            poll_until do
                writer.write(cmd)
                if (joints = reader.read_new)
                    (-0.01..0.01).include?(joints.elements[0].position) &&
                        (0.29..0.31).include?(joints.elements[1].position) &&
                        (0.39..0.41).include?(joints.elements[2].position) &&
                        (0.69..0.71).include?(joints.elements[3].position)
                end
            end
        end

        def initialize_joint_positions(writer)
            cmd = Types.base.samples.Joints.new(
                names: %w[m::j_00 m::j_01 m::child_j_00 m::child_j_01],
                elements: [Types.base.JointState.Position(0.05),
                            Types.base.JointState.Position(0.25),
                            Types.base.JointState.Position(0.45),
                            Types.base.JointState.Position(0.65)]
            )

            task.configure
            task.start
            writer.write(cmd)
        end
    end

    describe 'the joints export' do
        before do
            @task = gzserver 'joints_export.world', '/gazebo::w::m'
            @state_reader = @task.state_reader
        end

        it 'exports a set of joints in the specified order' do
            task.exported_joints = [Types.rock_gazebo.JointExport.new(
                port_name: 'test', prefix: '',
                joints: %w[m::child_j_00 m::j_01],
                port_period: Time.at(0)
            )]
            configure_start_and_read_one_new_sample 'test_samples' do |joints|
                joints.names == %w[m::child_j_00 m::j_01] &&
                    (0.39..0.51).include?(joints.elements[0].position) &&
                    (0.19..0.31).include?(joints.elements[1].position)
            end
        end

        it "exports values at the simulation rate" do
            task.exported_joints = [Types.rock_gazebo.JointExport.new(
                port_name: "test", prefix: "",
                joints: %w[m::child_j_00 m::j_01],
                port_period: Time.at(0)
            )]

            reader = create_active_reader("test_samples", type: :buffer, size: 100)

            samples = []
            poll_until(timeout: 10, message: "did not receive 25 samples") do
                while (s = reader.read_new)
                    samples << s
                end
                samples.size >= 25
            end
            @task.stop

            samples = samples.first(25)
            assert_equal 25, samples.size

            time_delta = samples.last.time - samples.first.time
            assert_in_delta 0.48, time_delta, 0.001
        end

        it "allows to control the output period of a joint export" do
            task.exported_joints = [Types.rock_gazebo.JointExport.new(
                port_name: "test", prefix: "",
                joints: %w[m::child_j_00 m::j_01],
                port_period: Time.at(0.1)
            )]

            reader = create_active_reader("test_samples", type: :buffer, size: 100)

            samples = []
            poll_until(timeout: 10, message: "did not receive 5 samples") do
                while (s = reader.read_new)
                    samples << s
                end
                samples.size >= 5
            end
            @task.stop

            samples = samples.first(5)
            assert_equal 5, samples.size

            time_delta = samples.last.time - samples.first.time
            assert_in_delta 0.4, time_delta, 0.01
        end

        it "handles joints given relatively to the model" do
            task.exported_joints = [Types.rock_gazebo.JointExport.new(
                port_name: "test", prefix: "",
                joints: %w[child_j_00 j_01],
                port_period: Time.at(0)
            )]
            configure_start_read_samples_and_stop "test_samples", 0.5 do |joints|
                joints.names == %w[m::child_j_00 m::j_01] &&
                    (0.39..0.51).include?(joints.elements[0].position) &&
                    (0.19..0.31).include?(joints.elements[1].position)
            end
        end

        it 'gives command access to a subset of the joints' do
            cmd = Types.base.samples.Joints.new(
                names: %w[m::child_j_00 m::j_01],
                elements: [Types.base.JointState.Effort(-1),
                            Types.base.JointState.Effort(1)]
            )

            task.exported_joints = [Types.rock_gazebo.JointExport.new(
                port_name: 'test', prefix: '',
                joints: %w[m::child_j_00 m::j_01],
                port_period: Time.at(0),
                ignore_joint_names: false
            )]

            task.configure
            task.start
            reader = task.port('test_samples').reader
            writer = task.port('test_cmd').writer
            poll_until do
                writer.write(cmd)
                if (joints = reader.read_new)
                    (0.39..0.41).include?(joints.elements[0].position) &&
                        (0.29..0.31).include?(joints.elements[1].position)
                end
            end
        end

        it 'validates the joint name size' do
            cmd = Types.base.samples.Joints.new(
                names: %w[m::child_j_01],
                elements: [Types.base.JointState.Effort(-1),
                            Types.base.JointState.Effort(1)]
            )

            task.exported_joints = [Types.rock_gazebo.JointExport.new(
                port_name: 'test', prefix: '',
                joints: %w[m::child_j_00 m::j_01],
                port_period: Time.at(0),
                ignore_joint_names: false
            )]

            task.configure
            task.start
            writer = task.port('test_cmd').writer
            writer.write(cmd)
            poll_until { @state_reader.read_new == :INVALID_JOINT_COMMAND }
        end

        it 'validates the joint names' do
            cmd = Types.base.samples.Joints.new(
                names: %w[m::child_j_01 m::j_02],
                elements: [Types.base.JointState.Effort(-1),
                            Types.base.JointState.Effort(1)]
            )

            task.exported_joints = [Types.rock_gazebo.JointExport.new(
                port_name: 'test', prefix: '',
                joints: %w[m::child_j_00 m::j_01],
                port_period: Time.at(0),
                ignore_joint_names: false
            )]

            task.configure
            task.start
            writer = task.port('test_cmd').writer
            writer.write(cmd)
            poll_until { @state_reader.read_new == :INVALID_JOINT_COMMAND }
        end

        it 'optionally ignores the names of the joints in the incoming command' do
            cmd = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.Effort(-1),
                            Types.base.JointState.Effort(1)]
            )

            task.exported_joints = [Types.rock_gazebo.JointExport.new(
                port_name: 'test', prefix: '',
                joints: %w[m::child_j_00 m::j_01],
                port_period: Time.at(0),
                ignore_joint_names: true
            )]

            task.configure
            task.start
            reader = task.port('test_samples').reader
            writer = task.port('test_cmd').writer
            poll_until do
                writer.write(cmd)
                if (joints = reader.read_new)
                    (0.39..0.41).include?(joints.elements[0].position) &&
                        (0.29..0.31).include?(joints.elements[1].position)
                end
            end
        end

        describe 'the prefix setting' do
            it 'raises if a joint name does not start with the requested prefix' do
                task.exported_joints = [Types.rock_gazebo.JointExport.new(
                    port_name: 'test', prefix: 'm::child_',
                    joints: %w[m::child_j_00 m::j_01],
                    port_period: Time.at(0)
                )]
                assert_raises(Orocos::StateTransitionFailed) do
                    task.configure
                end
            end
            it 'allows to remove a scope prefix from the joint names' do
                task.exported_joints = [Types.rock_gazebo.JointExport.new(
                    port_name: 'test', prefix: 'm::',
                    joints: %w[m::child_j_00 m::j_01],
                    port_period: Time.at(0)
                )]
                configure_start_and_read_one_new_sample 'test_samples' do |joints|
                    joints.names == %w[child_j_00 j_01] &&
                        (0.39..0.51).include?(joints.elements[0].position) &&
                        (0.19..0.31).include?(joints.elements[1].position)
                end
            end
        end

        it 'removes the ports on cleanup' do
            task.exported_joints = [Types.rock_gazebo.JointExport.new(
                port_name: 'test', prefix: '',
                joints: %w[m::child_j_00 m::j_01],
                port_period: Time.at(0)
            )]
            task.configure
            task.cleanup
            refute task.has_port?('test_cmd')
            refute task.has_port?('test_samples')
        end

        it 'fails to configure if a joint does not exist' do
            task.exported_joints = [Types.rock_gazebo.JointExport.new(
                port_name: 'test', prefix: '',
                joints: %w[does_not_exist m::j_01],
                port_period: Time.at(0)
            )]
            assert_raises(Orocos::StateTransitionFailed) do
                task.configure
            end
        end
    end

    describe 'the link export' do
        describe 'RBS export' do
            describe 'general properties' do
                before do
                    @task = gzserver 'model.world', '/gazebo::w::m'
                end

                it 'uses the link names as frames by default' do
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test', source_link: 'l', target_link: 'root',
                        port_period: Time.at(0)
                    )]
                    pose = configure_start_and_read_one_new_sample 'test'
                    assert_equal 'l', pose.sourceFrame
                    assert_equal 'root', pose.targetFrame
                end

                it 'allows to override the frame names' do
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test', source_link: 'l', target_link: 'root',
                        source_frame: 'src', target_frame: 'target',
                        port_period: Time.at(0)
                    )]
                    pose = configure_start_and_read_one_new_sample 'test'
                    assert_equal 'src', pose.sourceFrame
                    assert_equal 'target', pose.targetFrame
                end

                it 'sets cov_position from the provided cov_position' do
                    cov = matrix3_rand
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test', source_link: 'l', target_link: 'root',
                        cov_position: cov, port_period: Time.at(0)
                    )]
                    pose = configure_start_and_read_one_new_sample 'test'
                    assert_matrix3_in_delta cov, pose.cov_position, 1e-6
                end

                it 'sets cov_orientation from the provided cov_orientation' do
                    cov = matrix3_rand
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test', source_link: 'l', target_link: 'root',
                        cov_orientation: cov, port_period: Time.at(0)
                    )]
                    pose = configure_start_and_read_one_new_sample 'test'
                    assert_matrix3_in_delta cov, pose.cov_orientation, 1e-6
                end

                it 'sets cov_velocity from the provided cov_velocity' do
                    cov = matrix3_rand
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test', source_link: 'l', target_link: 'root',
                        cov_velocity: cov, port_period: Time.at(0)
                    )]
                    pose = configure_start_and_read_one_new_sample 'test'
                    assert_matrix3_in_delta cov, pose.cov_velocity, 1e-6
                end
            end

            describe 'pose' do
                before do
                    @task = gzserver 'model.world', '/gazebo::w::m'
                end

                it 'exports a link\'s pose' do
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test', source_link: 'l', target_link: 'root',
                        port_period: Time.at(0)
                    )]
                    link_pose = configure_start_and_read_one_new_sample 'test'
                    assert_eigen_approx Eigen::Vector3.new(2, 3, 4),
                                        link_pose.position
                    assert_eigen_approx(
                        Eigen::Quaternion.from_angle_axis(0.2, Eigen::Vector3.UnitZ),
                        link_pose.orientation
                    )
                end
            end

            describe 'velocity' do
                before do
                    @task = gzserver 'velocities.world', '/gazebo::w::m'
                end

                it 'exports a link\'s linear velocity' do
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test', source_link: 'l', target_link: 'root',
                        port_period: Time.at(0)
                    )]
                    rbs = configure_start_and_read_one_new_sample 'test' do |sample|
                        sample.velocity.z < -1
                    end
                    assert_in_delta 0, rbs.velocity.x, 1e-3
                    assert_in_delta 0, rbs.velocity.y, 1e-3
                end

                it 'exports the velocity in the target link\'s frame' do
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test',
                        source_link: 'root', target_link: 'rotated_link',
                        port_period: Time.at(0)
                    )]
                    rbs = configure_start_and_read_one_new_sample_matching 'test' do |sample|
                        sample.velocity.x > 1
                    end
                    assert_in_delta 0, rbs.velocity.y, 1e-3
                    assert_in_delta 0, rbs.velocity.z, 1e-3
                end

                it 'exports a link\'s angular velocity' do
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test', source_link: 'l', target_link: 'root',
                        port_period: Time.at(0)
                    )]
                    wrench = Types.base.samples.Wrench.new(
                        time: Time.now,
                        force: Eigen::Vector3.Zero,
                        torque: Eigen::Vector3.new(1, 0, 0)
                    )
                    @task.configure
                    @task.start
                    writer = @task.test_wrench.writer
                    rbs = assert_port_has_one_new_sample_matching 'test' do |sample|
                        writer.write(wrench)
                        sample.angular_velocity.x > 1
                    end
                    assert_in_delta 0, rbs.angular_velocity.y, 1e-3
                    assert_in_delta 0, rbs.angular_velocity.z, 1e-3
                end

                it 'exports the angular velocity always in the source '\
                    'links\'s frame' do
                    task.exported_links = [Types.rock_gazebo.LinkExport.new(
                        port_name: 'test',
                        source_link: 'root', target_link: 'rotated_link',
                        port_period: Time.at(0)
                    )]
                    wrench = Types.base.samples.Wrench.new(
                        time: Time.now,
                        force: Eigen::Vector3.Zero,
                        torque: Eigen::Vector3.new(1, 0, 0)
                    )
                    @task.configure
                    @task.start
                    writer = @task.test_wrench.writer
                    rbs = assert_port_has_one_new_sample_matching 'test' do |sample|
                        writer.write(wrench)
                        sample.angular_velocity.x > 1
                    end
                    assert_in_delta 0, rbs.angular_velocity.y, 1e-3
                    assert_in_delta 0, rbs.angular_velocity.z, 1e-3
                end
            end
        end

        describe 'acceleration export' do
            before do
                @task = gzserver 'accelerations.world', '/gazebo::w::m'
            end

            it 'exports a link\'s linear acceleration' do
                task.exported_links = [Types.rock_gazebo.LinkExport.new(
                    port_name: 'test', source_link: 'root',
                    port_period: Time.at(0)
                )]
                link_acceleration =
                    configure_start_and_read_one_new_sample 'test_acceleration'
                assert_in_delta 9.8, link_acceleration.acceleration.norm, 1e-3
            end

            it 'exports the linear acceleration expressed in the source frame' do
                task.exported_links = [Types.rock_gazebo.LinkExport.new(
                    port_name: 'test',
                    source_link: 'root', target_link: 'rotated_link',
                    port_period: Time.at(0)
                )]
                link_acceleration =
                    configure_start_and_read_one_new_sample 'test_acceleration'
                assert_in_delta(
                    -9.8, link_acceleration.acceleration.z, 1e-3,
                    "expected acceleration to be unaffected by the target frame, " \
                    "but it seems that it is"
                )
            end

            it 'exports a link\'s angular acceleration' do
                task.exported_links = [Types.rock_gazebo.LinkExport.new(
                    port_name: 'test', source_link: 'root', target_link: 'root',
                    port_period: Time.at(0)
                )]
                wrench = Types.base.samples.Wrench.new(
                    time: Time.now,
                    force: Eigen::Vector3.Zero,
                    torque: Eigen::Vector3.new(1, 0, 0)
                )

                task.configure
                task.start
                writer = task.test_wrench.writer

                acc = assert_port_has_one_new_sample_matching(
                    'test_acceleration'
                ) do |sample|
                    writer.write(wrench)
                    sample.angular_acceleration.norm > 0.1
                end
                assert_in_delta 0.5, acc.angular_acceleration.x, 1e-3
                assert_in_delta 0, acc.angular_acceleration.y, 1e-3
                assert_in_delta 0, acc.angular_acceleration.z, 1e-3
            end

            it 'exports the angular acceleration expressed in the source frame' do
                task.exported_links = [Types.rock_gazebo.LinkExport.new(
                    port_name: 'test',
                    source_link: 'root', target_link: 'rotated_link',
                    port_period: Time.at(0)
                )]
                wrench = Types.base.samples.Wrench.new(
                    time: Time.now,
                    force: Eigen::Vector3.Zero,
                    torque: Eigen::Vector3.new(1, 0, 0)
                )

                task.configure
                task.start
                writer = task.test_wrench.writer

                acc = assert_port_has_one_new_sample_matching(
                    'test_acceleration'
                ) do |sample|
                    writer.write(wrench)
                    sample.angular_acceleration.norm > 0.1
                end
                assert_in_delta 0.5, acc.angular_acceleration.x, 1e-3
                assert_in_delta 0, acc.angular_acceleration.y, 1e-3
                assert_in_delta 0, acc.angular_acceleration.z, 1e-3
            end
        end

        describe 'wrench export' do
            before do
                @task = gzserver 'wrench.world', '/gazebo::w::m'
                Orocos.load_typekit 'base'
                @task.wrench_command_timeout = Time.at(1)
            end

            it 'allows to send a force command to its source link' do
                task.exported_links = [Types.rock_gazebo.LinkExport.new(
                    port_name: 'test', source_link: 'leaf', target_link: 'root',
                    port_period: Time.at(0)
                )]
                task.configure
                task.start
                wrench_w = task.port('test_wrench').writer
                joints_r = task.port('joints_samples').reader

                wrench = Types.base.samples.Wrench.new(
                    time: Time.now,
                    force: Eigen::Vector3.new(0, 1, 0),
                    torque: Eigen::Vector3.new(0, 0, 0)
                )

                100.times do
                    wrench_w.write(wrench)
                    sleep 0.01
                end

                joints = assert_has_one_new_sample joints_r
                root2middle = joints.elements[0].position
                assert root2middle > 0.01,
                        'expected the root2middle joint position to be greater '\
                        "than 0.01 radians, but it is #{root2middle}"
                middle2leaf = joints.elements[1].position
                assert middle2leaf < -0.01,
                        'expected the middle2leaf joint position to be lower '\
                        "than -0.01 radians, but it is #{middle2leaf}"
            end

            it 'allows to send a torque command to its source link' do
                task.exported_links = [Types.rock_gazebo.LinkExport.new(
                    port_name: 'test', source_link: 'middle', target_link: 'root',
                    port_period: Time.at(0)
                )]
                task.configure
                task.start
                wrench_w = task.port('test_wrench').writer
                joints_r = task.port('joints_samples').reader

                wrench = Types.base.samples.Wrench.new(
                    time: Time.now,
                    force: Eigen::Vector3.new(0, 0, 0),
                    torque: Eigen::Vector3.new(0, 0, 1)
                )

                100.times do
                    wrench_w.write(wrench)
                    sleep 0.01
                end

                joints = assert_has_one_new_sample joints_r
                root2middle = joints.elements[0].position
                assert root2middle > 0.01,
                        'expected the root2middle joint position to be greater '\
                        "than 0.01 radians, but it is #{root2middle}"
            end
        end

        describe 'common features' do
            before do
                @task = gzserver 'model.world', '/gazebo::w::m'
                Orocos.load_typekit 'base'
            end

            it 'knows how to export a link in a nested model' do
                task.exported_links = [Types.rock_gazebo.LinkExport.new(
                    port_name: 'test', source_link: 'nested::l', target_link: 'root',
                    port_period: Time.at(0)
                )]
                task.configure
                task.start

                link_pose = assert_has_one_new_sample(task.test.reader)
                assert_eigen_approx Eigen::Vector3.new(3, 2, 1), link_pose.position, 1e-3
                assert_eigen_approx(
                    Eigen::Quaternion.from_angle_axis(-0.2, Eigen::Vector3.UnitZ),
                    link_pose.orientation
                )
            end

            it 'the pose\'s update period is controlled by the '\
                'port_period parameter' do
                task.exported_links = [Types.rock_gazebo.LinkExport.new(
                    port_name: 'test', source_link: 'l', target_link: 'root',
                    port_period: Time.at(0.1)
                )]
                task.configure
                task.start
                reader = task.test.reader
                first_pose = assert_has_one_new_sample(reader)
                second_pose = assert_has_one_new_sample(reader)
                assert_in_delta(second_pose.time - first_pose.time, 0.1, 0.025)
            end

            it 'refuses to configure if the source link does not exist' do
                task.exported_links = [Types.rock_gazebo.LinkExport.new(
                    port_name: 'test',
                    source_link: 'does_not_exist', target_link: 'root',
                    port_period: Time.at(0)
                )]
                assert_raises(Orocos::StateTransitionFailed) do
                    task.configure
                end
            end

            it 'refuses to configure if the target link does not exist' do
                task.exported_links = [Types.rock_gazebo.LinkExport.new(
                    port_name: 'test',
                    source_link: 'l', target_link: 'does_not_exist',
                    port_period: Time.at(0)
                )]
                assert_raises(Orocos::StateTransitionFailed) do
                    task.configure
                end
            end

            it 'refuses to configure if the port is already in use' do
                task.exported_links = [
                    Types.rock_gazebo.LinkExport.new(
                        port_name: 'test', source_link: 'l', target_link: 'root',
                        port_period: Time.at(0)
                    ),
                    Types.rock_gazebo.LinkExport.new(
                        port_name: 'test', source_link: 'l', target_link: 'root',
                        port_period: Time.at(0)
                    )
                ]
                assert_raises(Orocos::StateTransitionFailed) do
                    task.configure
                end
            end
        end
    end
end
