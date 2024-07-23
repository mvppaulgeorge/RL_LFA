// Benchmark "adder" written by ABC on Wed Jul 17 17:00:32 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n173, new_n174, new_n175, new_n176, new_n177,
    new_n178, new_n179, new_n180, new_n182, new_n183, new_n184, new_n185,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n195, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n258, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n273, new_n274, new_n275, new_n277, new_n278, new_n279,
    new_n280, new_n281, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n292, new_n293, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n327, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n353, new_n356, new_n358, new_n359, new_n360, new_n361, new_n363,
    new_n364;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor002aa1d24x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\a[2] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\b[1] ), .o1(new_n103));
  nand02aa1n03x5               g008(.a(new_n103), .b(new_n102), .o1(new_n104));
  nand42aa1n06x5               g009(.a(\b[0] ), .b(\a[1] ), .o1(new_n105));
  nand42aa1n06x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nand03aa1n03x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n107));
  nor002aa1d32x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  nand02aa1n04x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  nand42aa1n03x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nona23aa1n02x4               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  aoi012aa1n06x5               g017(.a(new_n108), .b(new_n110), .c(new_n109), .o1(new_n113));
  aoai13aa1n06x5               g018(.a(new_n113), .b(new_n112), .c(new_n104), .d(new_n107), .o1(new_n114));
  nanp02aa1n04x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nor002aa1d32x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nor022aa1n16x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nand42aa1n03x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  nona23aa1n02x4               g023(.a(new_n115), .b(new_n118), .c(new_n117), .d(new_n116), .out0(new_n119));
  nor042aa1d18x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  nand02aa1d16x5               g025(.a(\b[7] ), .b(\a[8] ), .o1(new_n121));
  norb02aa1n06x4               g026(.a(new_n121), .b(new_n120), .out0(new_n122));
  inv000aa1d42x5               g027(.a(\b[6] ), .o1(new_n123));
  nanb02aa1d36x5               g028(.a(\a[7] ), .b(new_n123), .out0(new_n124));
  nand22aa1n04x5               g029(.a(\b[6] ), .b(\a[7] ), .o1(new_n125));
  nano32aa1n03x7               g030(.a(new_n119), .b(new_n125), .c(new_n122), .d(new_n124), .out0(new_n126));
  inv000aa1d42x5               g031(.a(new_n120), .o1(new_n127));
  oai012aa1n06x5               g032(.a(new_n115), .b(new_n117), .c(new_n116), .o1(new_n128));
  nand22aa1n03x5               g033(.a(new_n125), .b(new_n121), .o1(new_n129));
  aoai13aa1n12x5               g034(.a(new_n127), .b(new_n129), .c(new_n128), .d(new_n124), .o1(new_n130));
  nand42aa1n10x5               g035(.a(\b[8] ), .b(\a[9] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n100), .out0(new_n132));
  aoai13aa1n03x5               g037(.a(new_n132), .b(new_n130), .c(new_n114), .d(new_n126), .o1(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n99), .b(new_n133), .c(new_n101), .out0(\s[10] ));
  nor002aa1d32x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  inv000aa1d42x5               g040(.a(new_n135), .o1(new_n136));
  nona22aa1n02x4               g041(.a(new_n133), .b(new_n100), .c(new_n97), .out0(new_n137));
  nand42aa1n04x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  aoi022aa1n02x5               g043(.a(new_n137), .b(new_n98), .c(new_n138), .d(new_n136), .o1(new_n139));
  aoi022aa1n06x5               g044(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n140));
  aoi013aa1n02x4               g045(.a(new_n139), .b(new_n137), .c(new_n136), .d(new_n140), .o1(\s[11] ));
  nanp03aa1n02x5               g046(.a(new_n137), .b(new_n136), .c(new_n140), .o1(new_n142));
  nor022aa1n16x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand42aa1n10x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanb02aa1n06x5               g049(.a(new_n143), .b(new_n144), .out0(new_n145));
  aoai13aa1n02x5               g050(.a(new_n145), .b(new_n135), .c(new_n137), .d(new_n140), .o1(new_n146));
  norb03aa1n03x5               g051(.a(new_n144), .b(new_n135), .c(new_n143), .out0(new_n147));
  aob012aa1n02x5               g052(.a(new_n146), .b(new_n142), .c(new_n147), .out0(\s[12] ));
  nanb02aa1n12x5               g053(.a(new_n135), .b(new_n138), .out0(new_n149));
  nano23aa1d15x5               g054(.a(new_n97), .b(new_n100), .c(new_n131), .d(new_n98), .out0(new_n150));
  nona22aa1d30x5               g055(.a(new_n150), .b(new_n145), .c(new_n149), .out0(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  aoai13aa1n02x5               g057(.a(new_n152), .b(new_n130), .c(new_n114), .d(new_n126), .o1(new_n153));
  oai022aa1n02x7               g058(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n154));
  oaoi03aa1n02x5               g059(.a(\a[12] ), .b(\b[11] ), .c(new_n136), .o1(new_n155));
  aoi013aa1n06x4               g060(.a(new_n155), .b(new_n147), .c(new_n140), .d(new_n154), .o1(new_n156));
  nor002aa1d32x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nand42aa1d28x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n158), .b(new_n157), .out0(new_n159));
  xnbna2aa1n03x5               g064(.a(new_n159), .b(new_n153), .c(new_n156), .out0(\s[13] ));
  inv000aa1d42x5               g065(.a(new_n157), .o1(new_n161));
  aob012aa1n03x5               g066(.a(new_n104), .b(new_n105), .c(new_n106), .out0(new_n162));
  norb02aa1n06x5               g067(.a(new_n109), .b(new_n108), .out0(new_n163));
  norb02aa1n06x5               g068(.a(new_n111), .b(new_n110), .out0(new_n164));
  nand43aa1n04x5               g069(.a(new_n162), .b(new_n163), .c(new_n164), .o1(new_n165));
  nanb02aa1n02x5               g070(.a(new_n116), .b(new_n115), .out0(new_n166));
  nanb02aa1n03x5               g071(.a(new_n117), .b(new_n118), .out0(new_n167));
  inv040aa1n02x5               g072(.a(new_n167), .o1(new_n168));
  nand02aa1n02x5               g073(.a(new_n124), .b(new_n125), .o1(new_n169));
  nona23aa1n09x5               g074(.a(new_n168), .b(new_n122), .c(new_n166), .d(new_n169), .out0(new_n170));
  aoi012aa1n02x7               g075(.a(new_n129), .b(new_n128), .c(new_n124), .o1(new_n171));
  nor042aa1n03x5               g076(.a(new_n171), .b(new_n120), .o1(new_n172));
  aoai13aa1n12x5               g077(.a(new_n172), .b(new_n170), .c(new_n165), .d(new_n113), .o1(new_n173));
  nand23aa1n03x5               g078(.a(new_n147), .b(new_n154), .c(new_n140), .o1(new_n174));
  oaib12aa1n06x5               g079(.a(new_n174), .b(new_n147), .c(new_n144), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n159), .b(new_n175), .c(new_n173), .d(new_n152), .o1(new_n176));
  nor002aa1n20x5               g081(.a(\b[13] ), .b(\a[14] ), .o1(new_n177));
  nand42aa1n20x5               g082(.a(\b[13] ), .b(\a[14] ), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  nona23aa1n02x4               g084(.a(new_n176), .b(new_n178), .c(new_n177), .d(new_n157), .out0(new_n180));
  aoai13aa1n02x5               g085(.a(new_n180), .b(new_n179), .c(new_n161), .d(new_n176), .o1(\s[14] ));
  nano23aa1d15x5               g086(.a(new_n157), .b(new_n177), .c(new_n178), .d(new_n158), .out0(new_n182));
  aoai13aa1n02x5               g087(.a(new_n182), .b(new_n175), .c(new_n173), .d(new_n152), .o1(new_n183));
  tech160nm_fioai012aa1n03p5x5 g088(.a(new_n178), .b(new_n177), .c(new_n157), .o1(new_n184));
  tech160nm_fixorc02aa1n05x5   g089(.a(\a[15] ), .b(\b[14] ), .out0(new_n185));
  xnbna2aa1n03x5               g090(.a(new_n185), .b(new_n183), .c(new_n184), .out0(\s[15] ));
  aobi12aa1n02x5               g091(.a(new_n185), .b(new_n183), .c(new_n184), .out0(new_n187));
  inv000aa1d42x5               g092(.a(\a[15] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\b[14] ), .o1(new_n189));
  inv000aa1d42x5               g094(.a(new_n182), .o1(new_n190));
  aoai13aa1n02x5               g095(.a(new_n184), .b(new_n190), .c(new_n153), .d(new_n156), .o1(new_n191));
  oaoi03aa1n02x5               g096(.a(new_n188), .b(new_n189), .c(new_n191), .o1(new_n192));
  tech160nm_fixorc02aa1n05x5   g097(.a(\a[16] ), .b(\b[15] ), .out0(new_n193));
  nanp02aa1n02x5               g098(.a(new_n189), .b(new_n188), .o1(new_n194));
  nanp02aa1n02x5               g099(.a(new_n193), .b(new_n194), .o1(new_n195));
  oai022aa1n02x5               g100(.a(new_n192), .b(new_n193), .c(new_n187), .d(new_n195), .o1(\s[16] ));
  nand23aa1d12x5               g101(.a(new_n182), .b(new_n185), .c(new_n193), .o1(new_n197));
  nor042aa1n12x5               g102(.a(new_n197), .b(new_n151), .o1(new_n198));
  aoai13aa1n12x5               g103(.a(new_n198), .b(new_n130), .c(new_n114), .d(new_n126), .o1(new_n199));
  inv000aa1d42x5               g104(.a(\a[16] ), .o1(new_n200));
  inv000aa1d42x5               g105(.a(\b[15] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(new_n201), .b(new_n200), .o1(new_n202));
  oai022aa1n02x5               g107(.a(new_n188), .b(new_n189), .c(new_n201), .d(new_n200), .o1(new_n203));
  aoai13aa1n06x5               g108(.a(new_n202), .b(new_n203), .c(new_n184), .d(new_n194), .o1(new_n204));
  aoib12aa1n12x5               g109(.a(new_n204), .b(new_n175), .c(new_n197), .out0(new_n205));
  nor042aa1n04x5               g110(.a(\b[16] ), .b(\a[17] ), .o1(new_n206));
  nand42aa1n20x5               g111(.a(\b[16] ), .b(\a[17] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  xnbna2aa1n03x5               g113(.a(new_n208), .b(new_n199), .c(new_n205), .out0(\s[17] ));
  nanp02aa1n09x5               g114(.a(new_n199), .b(new_n205), .o1(new_n210));
  aoi012aa1n02x5               g115(.a(new_n206), .b(new_n210), .c(new_n207), .o1(new_n211));
  nor042aa1n04x5               g116(.a(\b[17] ), .b(\a[18] ), .o1(new_n212));
  nand02aa1d28x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  norb02aa1n02x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  oaih22aa1d12x5               g119(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n215));
  aoi122aa1n06x5               g120(.a(new_n215), .b(\b[17] ), .c(\a[18] ), .d(new_n210), .e(new_n208), .o1(new_n216));
  oabi12aa1n03x5               g121(.a(new_n216), .b(new_n211), .c(new_n214), .out0(\s[18] ));
  oabi12aa1n06x5               g122(.a(new_n204), .b(new_n156), .c(new_n197), .out0(new_n218));
  nano23aa1d15x5               g123(.a(new_n206), .b(new_n212), .c(new_n213), .d(new_n207), .out0(new_n219));
  aoai13aa1n06x5               g124(.a(new_n219), .b(new_n218), .c(new_n173), .d(new_n198), .o1(new_n220));
  oai012aa1n02x5               g125(.a(new_n213), .b(new_n212), .c(new_n206), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[19] ), .b(\b[18] ), .out0(new_n222));
  xnbna2aa1n03x5               g127(.a(new_n222), .b(new_n220), .c(new_n221), .out0(\s[19] ));
  xnrc02aa1n02x5               g128(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aobi12aa1n02x5               g129(.a(new_n222), .b(new_n220), .c(new_n221), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n219), .o1(new_n226));
  aoai13aa1n03x5               g131(.a(new_n221), .b(new_n226), .c(new_n199), .d(new_n205), .o1(new_n227));
  inv040aa1d32x5               g132(.a(\a[19] ), .o1(new_n228));
  inv040aa1d28x5               g133(.a(\b[18] ), .o1(new_n229));
  nand22aa1n12x5               g134(.a(new_n229), .b(new_n228), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  nand02aa1n06x5               g136(.a(\b[18] ), .b(\a[19] ), .o1(new_n232));
  nor042aa1n06x5               g137(.a(\b[19] ), .b(\a[20] ), .o1(new_n233));
  tech160nm_finand02aa1n05x5   g138(.a(\b[19] ), .b(\a[20] ), .o1(new_n234));
  nanb02aa1n02x5               g139(.a(new_n233), .b(new_n234), .out0(new_n235));
  aoai13aa1n03x5               g140(.a(new_n235), .b(new_n231), .c(new_n227), .d(new_n232), .o1(new_n236));
  nano22aa1n02x4               g141(.a(new_n233), .b(new_n230), .c(new_n234), .out0(new_n237));
  oaib12aa1n03x5               g142(.a(new_n236), .b(new_n225), .c(new_n237), .out0(\s[20] ));
  nano22aa1n03x7               g143(.a(new_n235), .b(new_n230), .c(new_n232), .out0(new_n239));
  nand22aa1n12x5               g144(.a(new_n239), .b(new_n219), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  aoai13aa1n03x5               g146(.a(new_n241), .b(new_n218), .c(new_n173), .d(new_n198), .o1(new_n242));
  nanb03aa1n09x5               g147(.a(new_n233), .b(new_n234), .c(new_n232), .out0(new_n243));
  nand23aa1n03x5               g148(.a(new_n215), .b(new_n230), .c(new_n213), .o1(new_n244));
  aoai13aa1n06x5               g149(.a(new_n234), .b(new_n233), .c(new_n228), .d(new_n229), .o1(new_n245));
  oai012aa1n18x5               g150(.a(new_n245), .b(new_n244), .c(new_n243), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  nor042aa1n06x5               g152(.a(\b[20] ), .b(\a[21] ), .o1(new_n248));
  nanp02aa1n04x5               g153(.a(\b[20] ), .b(\a[21] ), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n249), .b(new_n248), .out0(new_n250));
  xnbna2aa1n03x5               g155(.a(new_n250), .b(new_n242), .c(new_n247), .out0(\s[21] ));
  aobi12aa1n02x5               g156(.a(new_n250), .b(new_n242), .c(new_n247), .out0(new_n252));
  aoai13aa1n02x5               g157(.a(new_n247), .b(new_n240), .c(new_n199), .d(new_n205), .o1(new_n253));
  aoi012aa1n03x5               g158(.a(new_n248), .b(new_n253), .c(new_n249), .o1(new_n254));
  nor042aa1n06x5               g159(.a(\b[21] ), .b(\a[22] ), .o1(new_n255));
  nand42aa1n04x5               g160(.a(\b[21] ), .b(\a[22] ), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n256), .b(new_n255), .out0(new_n257));
  nona22aa1n02x4               g162(.a(new_n256), .b(new_n255), .c(new_n248), .out0(new_n258));
  oai022aa1n02x5               g163(.a(new_n254), .b(new_n257), .c(new_n258), .d(new_n252), .o1(\s[22] ));
  nano23aa1d15x5               g164(.a(new_n248), .b(new_n255), .c(new_n256), .d(new_n249), .out0(new_n260));
  and003aa1n02x5               g165(.a(new_n239), .b(new_n219), .c(new_n260), .o(new_n261));
  aoai13aa1n06x5               g166(.a(new_n261), .b(new_n218), .c(new_n173), .d(new_n198), .o1(new_n262));
  inv000aa1n02x5               g167(.a(new_n261), .o1(new_n263));
  nano22aa1n02x4               g168(.a(new_n233), .b(new_n232), .c(new_n234), .out0(new_n264));
  oai012aa1n02x5               g169(.a(new_n213), .b(\b[18] ), .c(\a[19] ), .o1(new_n265));
  oab012aa1n06x5               g170(.a(new_n265), .b(new_n206), .c(new_n212), .out0(new_n266));
  inv030aa1n02x5               g171(.a(new_n245), .o1(new_n267));
  aoai13aa1n06x5               g172(.a(new_n260), .b(new_n267), .c(new_n266), .d(new_n264), .o1(new_n268));
  oa0012aa1n02x5               g173(.a(new_n256), .b(new_n255), .c(new_n248), .o(new_n269));
  inv020aa1n03x5               g174(.a(new_n269), .o1(new_n270));
  nand02aa1n03x5               g175(.a(new_n268), .b(new_n270), .o1(new_n271));
  inv000aa1n02x5               g176(.a(new_n271), .o1(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n263), .c(new_n199), .d(new_n205), .o1(new_n273));
  xorc02aa1n12x5               g178(.a(\a[23] ), .b(\b[22] ), .out0(new_n274));
  aoi112aa1n02x5               g179(.a(new_n274), .b(new_n269), .c(new_n246), .d(new_n260), .o1(new_n275));
  aoi022aa1n02x5               g180(.a(new_n273), .b(new_n274), .c(new_n262), .d(new_n275), .o1(\s[23] ));
  aobi12aa1n02x5               g181(.a(new_n274), .b(new_n262), .c(new_n272), .out0(new_n277));
  norp02aa1n02x5               g182(.a(\b[22] ), .b(\a[23] ), .o1(new_n278));
  xnrc02aa1n12x5               g183(.a(\b[23] ), .b(\a[24] ), .out0(new_n279));
  aoai13aa1n03x5               g184(.a(new_n279), .b(new_n278), .c(new_n273), .d(new_n274), .o1(new_n280));
  norp02aa1n02x5               g185(.a(new_n279), .b(new_n278), .o1(new_n281));
  oaib12aa1n03x5               g186(.a(new_n280), .b(new_n277), .c(new_n281), .out0(\s[24] ));
  norb02aa1n03x5               g187(.a(new_n274), .b(new_n279), .out0(new_n283));
  nanb03aa1n02x5               g188(.a(new_n240), .b(new_n283), .c(new_n260), .out0(new_n284));
  inv040aa1n02x5               g189(.a(new_n283), .o1(new_n285));
  orn002aa1n02x5               g190(.a(\a[23] ), .b(\b[22] ), .o(new_n286));
  oao003aa1n02x5               g191(.a(\a[24] ), .b(\b[23] ), .c(new_n286), .carry(new_n287));
  aoai13aa1n06x5               g192(.a(new_n287), .b(new_n285), .c(new_n268), .d(new_n270), .o1(new_n288));
  inv040aa1n03x5               g193(.a(new_n288), .o1(new_n289));
  aoai13aa1n04x5               g194(.a(new_n289), .b(new_n284), .c(new_n199), .d(new_n205), .o1(new_n290));
  xorb03aa1n02x5               g195(.a(new_n290), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g196(.a(\b[24] ), .b(\a[25] ), .o1(new_n292));
  xnrc02aa1n12x5               g197(.a(\b[24] ), .b(\a[25] ), .out0(new_n293));
  inv000aa1d42x5               g198(.a(new_n293), .o1(new_n294));
  tech160nm_fixnrc02aa1n05x5   g199(.a(\b[25] ), .b(\a[26] ), .out0(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n292), .c(new_n290), .d(new_n294), .o1(new_n296));
  norp02aa1n02x5               g201(.a(new_n295), .b(new_n292), .o1(new_n297));
  aob012aa1n03x5               g202(.a(new_n297), .b(new_n290), .c(new_n294), .out0(new_n298));
  nanp02aa1n03x5               g203(.a(new_n296), .b(new_n298), .o1(\s[26] ));
  nor042aa1n02x5               g204(.a(new_n295), .b(new_n293), .o1(new_n300));
  inv000aa1n06x5               g205(.a(new_n300), .o1(new_n301));
  nano23aa1d15x5               g206(.a(new_n240), .b(new_n301), .c(new_n283), .d(new_n260), .out0(new_n302));
  aoai13aa1n06x5               g207(.a(new_n302), .b(new_n218), .c(new_n173), .d(new_n198), .o1(new_n303));
  nanp02aa1n02x5               g208(.a(\b[25] ), .b(\a[26] ), .o1(new_n304));
  inv000aa1n02x5               g209(.a(new_n297), .o1(new_n305));
  aoi022aa1n12x5               g210(.a(new_n288), .b(new_n300), .c(new_n304), .d(new_n305), .o1(new_n306));
  xorc02aa1n12x5               g211(.a(\a[27] ), .b(\b[26] ), .out0(new_n307));
  xnbna2aa1n06x5               g212(.a(new_n307), .b(new_n306), .c(new_n303), .out0(\s[27] ));
  norp02aa1n02x5               g213(.a(\b[26] ), .b(\a[27] ), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n309), .o1(new_n310));
  aoai13aa1n06x5               g215(.a(new_n283), .b(new_n269), .c(new_n246), .d(new_n260), .o1(new_n311));
  oai012aa1n02x5               g216(.a(new_n304), .b(new_n295), .c(new_n292), .o1(new_n312));
  aoai13aa1n06x5               g217(.a(new_n312), .b(new_n301), .c(new_n311), .d(new_n287), .o1(new_n313));
  aoai13aa1n03x5               g218(.a(new_n307), .b(new_n313), .c(new_n210), .d(new_n302), .o1(new_n314));
  xorc02aa1n02x5               g219(.a(\a[28] ), .b(\b[27] ), .out0(new_n315));
  inv000aa1d42x5               g220(.a(new_n307), .o1(new_n316));
  oai022aa1d18x5               g221(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n317));
  aoi012aa1n02x5               g222(.a(new_n317), .b(\a[28] ), .c(\b[27] ), .o1(new_n318));
  aoai13aa1n04x5               g223(.a(new_n318), .b(new_n316), .c(new_n306), .d(new_n303), .o1(new_n319));
  aoai13aa1n03x5               g224(.a(new_n319), .b(new_n315), .c(new_n314), .d(new_n310), .o1(\s[28] ));
  and002aa1n02x5               g225(.a(new_n315), .b(new_n307), .o(new_n321));
  aoai13aa1n03x5               g226(.a(new_n321), .b(new_n313), .c(new_n210), .d(new_n302), .o1(new_n322));
  inv000aa1d42x5               g227(.a(new_n321), .o1(new_n323));
  aob012aa1n12x5               g228(.a(new_n317), .b(\b[27] ), .c(\a[28] ), .out0(new_n324));
  aoai13aa1n04x5               g229(.a(new_n324), .b(new_n323), .c(new_n306), .d(new_n303), .o1(new_n325));
  xorc02aa1n02x5               g230(.a(\a[29] ), .b(\b[28] ), .out0(new_n326));
  norb02aa1n02x5               g231(.a(new_n324), .b(new_n326), .out0(new_n327));
  aoi022aa1n02x7               g232(.a(new_n325), .b(new_n326), .c(new_n322), .d(new_n327), .o1(\s[29] ));
  xorb03aa1n02x5               g233(.a(new_n105), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g234(.a(new_n316), .b(new_n315), .c(new_n326), .out0(new_n330));
  aoai13aa1n03x5               g235(.a(new_n330), .b(new_n313), .c(new_n210), .d(new_n302), .o1(new_n331));
  inv000aa1d42x5               g236(.a(new_n330), .o1(new_n332));
  tech160nm_fioaoi03aa1n03p5x5 g237(.a(\a[29] ), .b(\b[28] ), .c(new_n324), .o1(new_n333));
  inv000aa1d42x5               g238(.a(new_n333), .o1(new_n334));
  aoai13aa1n03x5               g239(.a(new_n334), .b(new_n332), .c(new_n306), .d(new_n303), .o1(new_n335));
  xorc02aa1n02x5               g240(.a(\a[30] ), .b(\b[29] ), .out0(new_n336));
  norp02aa1n02x5               g241(.a(\b[28] ), .b(\a[29] ), .o1(new_n337));
  aoi012aa1n02x5               g242(.a(new_n324), .b(\a[29] ), .c(\b[28] ), .o1(new_n338));
  norp03aa1n02x5               g243(.a(new_n338), .b(new_n336), .c(new_n337), .o1(new_n339));
  aoi022aa1n03x5               g244(.a(new_n335), .b(new_n336), .c(new_n331), .d(new_n339), .o1(\s[30] ));
  nano32aa1n03x7               g245(.a(new_n316), .b(new_n336), .c(new_n315), .d(new_n326), .out0(new_n341));
  aoai13aa1n03x5               g246(.a(new_n341), .b(new_n313), .c(new_n210), .d(new_n302), .o1(new_n342));
  xorc02aa1n02x5               g247(.a(\a[31] ), .b(\b[30] ), .out0(new_n343));
  inv000aa1d42x5               g248(.a(\a[30] ), .o1(new_n344));
  inv000aa1d42x5               g249(.a(\b[29] ), .o1(new_n345));
  oabi12aa1n02x5               g250(.a(new_n343), .b(\a[30] ), .c(\b[29] ), .out0(new_n346));
  oaoi13aa1n04x5               g251(.a(new_n346), .b(new_n333), .c(new_n344), .d(new_n345), .o1(new_n347));
  inv000aa1d42x5               g252(.a(new_n341), .o1(new_n348));
  oaoi03aa1n02x5               g253(.a(new_n344), .b(new_n345), .c(new_n333), .o1(new_n349));
  aoai13aa1n04x5               g254(.a(new_n349), .b(new_n348), .c(new_n306), .d(new_n303), .o1(new_n350));
  aoi022aa1n03x5               g255(.a(new_n350), .b(new_n343), .c(new_n342), .d(new_n347), .o1(\s[31] ));
  xnbna2aa1n03x5               g256(.a(new_n164), .b(new_n107), .c(new_n104), .out0(\s[3] ));
  aoi112aa1n02x5               g257(.a(new_n110), .b(new_n163), .c(new_n162), .d(new_n164), .o1(new_n353));
  oaoi13aa1n02x5               g258(.a(new_n353), .b(new_n114), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xnbna2aa1n03x5               g259(.a(new_n168), .b(new_n165), .c(new_n113), .out0(\s[5] ));
  tech160nm_fiao0012aa1n03p5x5 g260(.a(new_n117), .b(new_n114), .c(new_n118), .o(new_n356));
  xorb03aa1n02x5               g261(.a(new_n356), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g262(.a(new_n169), .b(new_n116), .c(new_n356), .d(new_n115), .o1(new_n358));
  nanb02aa1n02x5               g263(.a(new_n166), .b(new_n356), .out0(new_n359));
  nano22aa1n02x4               g264(.a(new_n116), .b(new_n124), .c(new_n125), .out0(new_n360));
  nanp02aa1n03x5               g265(.a(new_n359), .b(new_n360), .o1(new_n361));
  nanp02aa1n02x5               g266(.a(new_n361), .b(new_n358), .o1(\s[7] ));
  oaib12aa1n02x5               g267(.a(new_n125), .b(new_n120), .c(new_n121), .out0(new_n363));
  aob012aa1n02x5               g268(.a(new_n122), .b(new_n361), .c(new_n125), .out0(new_n364));
  oaib12aa1n03x5               g269(.a(new_n364), .b(new_n363), .c(new_n361), .out0(\s[8] ));
  xorb03aa1n02x5               g270(.a(new_n173), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


