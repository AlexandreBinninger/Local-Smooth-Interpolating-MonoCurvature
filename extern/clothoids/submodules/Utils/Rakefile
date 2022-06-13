%w(colorize rake fileutils).each do |gem|
  begin
    require gem
  rescue LoadError
    warn "Install the #{gem} gem:\n $ (sudo) gem install #{gem}".magenta
    exit 1
  end
end

case RUBY_PLATFORM
when /darwin/
  OS = :mac
when /linux/
  OS = :linux
when /cygwin|mswin|mingw|bccwin|wince|emx/
  OS = :win
end

require_relative "./Rakefile_common.rb"

file_base = File.expand_path(File.dirname(__FILE__)).to_s

cmd_cmake_build = ""
if COMPILE_EXECUTABLE then
  cmd_cmake_build += ' -DEB_ENABLE_TESTS:VAR=ON '
else
  cmd_cmake_build += ' -DEB_ENABLE_TESTS:VAR=OFF '
end
if COMPILE_DYNAMIC then
  cmd_cmake_build += ' -DEB_BUILD_SHARED:VAR=ON '
else
  cmd_cmake_build += ' -DEB_BUILD_SHARED:VAR=OFF '
end
if COMPILE_DEBUG then
  cmd_cmake_build += ' -DCMAKE_BUILD_TYPE:VAR=Debug --loglevel=STATUS '
else
  cmd_cmake_build += ' -DCMAKE_BUILD_TYPE:VAR=Release --loglevel=STATUS '
end

desc "default task --> build"
task :default => :build

TESTS = [
  "test_Malloc",
  "test_Quaternion",
  "test_rang",
  "test_table",
  "test_Thread",
  "test_trace",
  "test_zstr"
]

desc "run tests"
task :run do
  puts "UTILS run tests".green
  case OS
  when :mac,:linux
    TESTS.each do |cmd|
      exe = "./bin/#{cmd}"
      next unless File.exist?(exe)
      puts "execute #{exe}".yellow
      sh exe
    end
  when :win
    TESTS.each do |cmd|
      exe = "bin\\#{cmd}.exe"
      next unless File.exist?(exe)
      puts "execute #{exe}".yellow
      sh exe
    end
  end
end

desc "build lib"
task :build do
  puts "UTILS build".green
  case OS
  when :mac
    Rake::Task[:build_osx].invoke
  when :linux
    Rake::Task[:build_linux].invoke
  when :win
    Rake::Task[:build_win].invoke
  end
end

desc "compile for Visual Studio [default year=2017, bits=x64]"
task :build_win, [:year, :bits] do |t, args|

  args.with_defaults( :year => "2017", :bits => "x64" )

  puts "UTILS build on windows".green

  FileUtils.cd "ThirdParties"
    sh "rake install_win[#{args.year},#{args.bits}]"
  FileUtils.cd ".."

  FileUtils.rm_rf 'lib'

  dir = "vs_#{args.year}_#{args.bits}"

  FileUtils.rm_rf   dir
  FileUtils.mkdir_p dir
  FileUtils.cd      dir

  FileUtils.mkdir_p "../lib/lib"
  FileUtils.mkdir_p "../lib/bin"
  FileUtils.mkdir_p "../lib/bin/"+args.bits
  FileUtils.mkdir_p "../lib/dll"
  FileUtils.mkdir_p "../lib/include"

  cmd_cmake = win_vs(args.bits,args.year) + cmd_cmake_build

  puts "run CMAKE for UTILS".yellow
  sh cmd_cmake + ' ..'
  puts "compile with CMAKE for UTILS".yellow
  if COMPILE_DEBUG then
    sh 'cmake --build . --config Debug --target install '+PARALLEL+QUIET
  else
    sh 'cmake --build . --config Release --target install '+PARALLEL+QUIET
  end

  if RUN_CPACK then
    puts "run CPACK for UTILS".yellow
    sh 'cpack -C CPackConfig.cmake'
    sh 'cpack -C CPackSourceConfig.cmake'
  end

  FileUtils.cd '..'
end

task :build_osx_linux do
  puts "UTILS build (osx/linux)".green

  FileUtils.cd "ThirdParties"
    sh "rake install"
  FileUtils.cd ".."

  FileUtils.rm_rf 'lib'

  dir = "build"

  FileUtils.rm_rf   dir
  FileUtils.mkdir_p dir
  FileUtils.cd      dir

  cmd_cmake = "cmake " + cmd_cmake_build

  puts "run CMAKE for UTILS".yellow
  sh cmd_cmake + ' ..'
  puts "compile with CMAKE for UTILS".yellow
  if COMPILE_DEBUG then
    sh 'cmake --build . --config Debug --target install '+PARALLEL+QUIET
  else
    sh 'cmake --build . --config Release --target install '+PARALLEL+QUIET
  end

  FileUtils.cd '..'
end

desc 'compile for OSX'
task :build_osx => :build_osx_linux do
  FileUtils.cd "build"
  if RUN_CPACK then
    puts "run CPACK for UTILS".yellow
    sh 'cpack -C CPackConfig.cmake'
    sh 'cpack -C CPackSourceConfig.cmake'
  end
  FileUtils.cd ".."
end

desc 'compile for LINUX'
task :build_linux => :build_osx_linux do
  FileUtils.cd "build"
  if RUN_CPACK then
    puts "run CPACK for UTILS".yellow
    sh 'cpack -C CPackConfig.cmake'
    sh 'cpack -C CPackSourceConfig.cmake'
  end
  FileUtils.cd ".."
end

desc "clean for OSX"
task :clean_osx do
  FileUtils.rm_rf 'lib'
end

desc "clean for LINUX"
task :clean_linux => :clean_osx

desc "clean for WINDOWS"
task :clean_win do
  FileUtils.rm_rf 'lib'
  FileUtils.rm_rf 'vs_*'
end
