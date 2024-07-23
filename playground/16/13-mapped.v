// Benchmark "adder" written by ABC on Wed Jul 17 20:14:59 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n168, new_n169, new_n170,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n195, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n205, new_n206, new_n207, new_n208, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n252, new_n253, new_n254, new_n255, new_n256, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n337, new_n340,
    new_n341, new_n343, new_n344, new_n346;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  and002aa1n02x5               g004(.a(\b[3] ), .b(\a[4] ), .o(new_n100));
  inv000aa1d42x5               g005(.a(\a[3] ), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\b[2] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(new_n102), .b(new_n101), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(new_n103), .b(new_n104), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nor042aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nand22aa1n06x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  oai012aa1n06x5               g013(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n109));
  oa0022aa1n02x5               g014(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n110));
  oaoi13aa1n02x5               g015(.a(new_n100), .b(new_n110), .c(new_n109), .d(new_n105), .o1(new_n111));
  nor022aa1n04x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand42aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nand42aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nor022aa1n04x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nona23aa1n09x5               g020(.a(new_n114), .b(new_n113), .c(new_n115), .d(new_n112), .out0(new_n116));
  xnrc02aa1n03x5               g021(.a(\b[5] ), .b(\a[6] ), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .out0(new_n118));
  nor043aa1n02x5               g023(.a(new_n116), .b(new_n117), .c(new_n118), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\a[6] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\b[5] ), .o1(new_n121));
  norp02aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(new_n120), .b(new_n121), .c(new_n122), .o1(new_n123));
  tech160nm_fiao0012aa1n02p5x5 g028(.a(new_n112), .b(new_n115), .c(new_n113), .o(new_n124));
  oabi12aa1n02x5               g029(.a(new_n124), .b(new_n116), .c(new_n123), .out0(new_n125));
  xorc02aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n125), .c(new_n111), .d(new_n119), .o1(new_n127));
  nor002aa1d32x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand42aa1d28x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n02x7               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g036(.a(new_n128), .o1(new_n132));
  inv000aa1d42x5               g037(.a(new_n129), .o1(new_n133));
  nor042aa1n09x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  inv000aa1d42x5               g041(.a(new_n136), .o1(new_n137));
  aoi113aa1n03x7               g042(.a(new_n137), .b(new_n133), .c(new_n127), .d(new_n132), .e(new_n99), .o1(new_n138));
  oai012aa1n02x7               g043(.a(new_n110), .b(new_n109), .c(new_n105), .o1(new_n139));
  nor002aa1n02x5               g044(.a(new_n118), .b(new_n117), .o1(new_n140));
  nona23aa1n09x5               g045(.a(new_n139), .b(new_n140), .c(new_n116), .d(new_n100), .out0(new_n141));
  oab012aa1n12x5               g046(.a(new_n124), .b(new_n116), .c(new_n123), .out0(new_n142));
  and002aa1n18x5               g047(.a(\b[8] ), .b(\a[9] ), .o(new_n143));
  aoai13aa1n02x5               g048(.a(new_n99), .b(new_n143), .c(new_n141), .d(new_n142), .o1(new_n144));
  aoai13aa1n12x5               g049(.a(new_n129), .b(new_n128), .c(new_n97), .d(new_n98), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  aoi112aa1n02x5               g051(.a(new_n146), .b(new_n136), .c(new_n144), .d(new_n130), .o1(new_n147));
  norp02aa1n02x5               g052(.a(new_n147), .b(new_n138), .o1(\s[11] ));
  inv000aa1d42x5               g053(.a(new_n134), .o1(new_n149));
  nor042aa1n02x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  norb02aa1n02x5               g056(.a(new_n151), .b(new_n150), .out0(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  nano22aa1n02x4               g058(.a(new_n138), .b(new_n149), .c(new_n153), .out0(new_n154));
  aoai13aa1n02x5               g059(.a(new_n136), .b(new_n146), .c(new_n144), .d(new_n130), .o1(new_n155));
  aoi012aa1n02x5               g060(.a(new_n153), .b(new_n155), .c(new_n149), .o1(new_n156));
  norp02aa1n02x5               g061(.a(new_n156), .b(new_n154), .o1(\s[12] ));
  inv000aa1d42x5               g062(.a(new_n143), .o1(new_n158));
  aoi112aa1n02x5               g063(.a(new_n133), .b(new_n128), .c(new_n97), .d(new_n98), .o1(new_n159));
  nano23aa1n02x4               g064(.a(new_n134), .b(new_n150), .c(new_n151), .d(new_n135), .out0(new_n160));
  nanp03aa1n02x5               g065(.a(new_n160), .b(new_n158), .c(new_n159), .o1(new_n161));
  nona23aa1n08x5               g066(.a(new_n151), .b(new_n135), .c(new_n134), .d(new_n150), .out0(new_n162));
  tech160nm_fioai012aa1n03p5x5 g067(.a(new_n151), .b(new_n150), .c(new_n134), .o1(new_n163));
  oai012aa1d24x5               g068(.a(new_n163), .b(new_n162), .c(new_n145), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n161), .c(new_n141), .d(new_n142), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g072(.a(\a[13] ), .o1(new_n168));
  inv000aa1d42x5               g073(.a(\b[12] ), .o1(new_n169));
  oaoi03aa1n02x5               g074(.a(new_n168), .b(new_n169), .c(new_n166), .o1(new_n170));
  xnrb03aa1n02x5               g075(.a(new_n170), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nano32aa1n02x5               g076(.a(new_n162), .b(new_n130), .c(new_n158), .d(new_n99), .out0(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n125), .c(new_n111), .d(new_n119), .o1(new_n173));
  nor022aa1n16x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nand42aa1n04x5               g079(.a(\b[13] ), .b(\a[14] ), .o1(new_n175));
  aoai13aa1n09x5               g080(.a(new_n175), .b(new_n174), .c(new_n168), .d(new_n169), .o1(new_n176));
  norp02aa1n02x5               g081(.a(\b[12] ), .b(\a[13] ), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(\b[12] ), .b(\a[13] ), .o1(new_n178));
  nona23aa1n02x4               g083(.a(new_n175), .b(new_n178), .c(new_n177), .d(new_n174), .out0(new_n179));
  aoai13aa1n04x5               g084(.a(new_n176), .b(new_n179), .c(new_n173), .d(new_n165), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor022aa1n16x5               g086(.a(\b[14] ), .b(\a[15] ), .o1(new_n182));
  nanp02aa1n02x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  nanb02aa1n02x5               g088(.a(new_n182), .b(new_n183), .out0(new_n184));
  inv000aa1n02x5               g089(.a(new_n184), .o1(new_n185));
  nor042aa1n02x5               g090(.a(\b[15] ), .b(\a[16] ), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(\b[15] ), .b(\a[16] ), .o1(new_n187));
  nanb02aa1n06x5               g092(.a(new_n186), .b(new_n187), .out0(new_n188));
  inv000aa1d42x5               g093(.a(new_n188), .o1(new_n189));
  aoi112aa1n02x5               g094(.a(new_n189), .b(new_n182), .c(new_n180), .d(new_n185), .o1(new_n190));
  inv000aa1d42x5               g095(.a(new_n182), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n176), .o1(new_n192));
  nano23aa1n02x4               g097(.a(new_n177), .b(new_n174), .c(new_n175), .d(new_n178), .out0(new_n193));
  aoai13aa1n02x5               g098(.a(new_n185), .b(new_n192), .c(new_n166), .d(new_n193), .o1(new_n194));
  aoi012aa1n02x5               g099(.a(new_n188), .b(new_n194), .c(new_n191), .o1(new_n195));
  norp02aa1n02x5               g100(.a(new_n195), .b(new_n190), .o1(\s[16] ));
  nona23aa1n03x5               g101(.a(new_n187), .b(new_n183), .c(new_n182), .d(new_n186), .out0(new_n197));
  nor042aa1n02x5               g102(.a(new_n197), .b(new_n179), .o1(new_n198));
  nand02aa1n02x5               g103(.a(new_n172), .b(new_n198), .o1(new_n199));
  aoi012aa1n02x5               g104(.a(new_n186), .b(new_n182), .c(new_n187), .o1(new_n200));
  tech160nm_fioai012aa1n04x5   g105(.a(new_n200), .b(new_n197), .c(new_n176), .o1(new_n201));
  aoi012aa1d18x5               g106(.a(new_n201), .b(new_n164), .c(new_n198), .o1(new_n202));
  aoai13aa1n12x5               g107(.a(new_n202), .b(new_n199), .c(new_n141), .d(new_n142), .o1(new_n203));
  xorb03aa1n02x5               g108(.a(new_n203), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g109(.a(\a[18] ), .o1(new_n205));
  nor042aa1d18x5               g110(.a(\b[16] ), .b(\a[17] ), .o1(new_n206));
  nand42aa1n03x5               g111(.a(\b[16] ), .b(\a[17] ), .o1(new_n207));
  tech160nm_fiaoi012aa1n05x5   g112(.a(new_n206), .b(new_n203), .c(new_n207), .o1(new_n208));
  xorb03aa1n02x5               g113(.a(new_n208), .b(\b[17] ), .c(new_n205), .out0(\s[18] ));
  nano32aa1n03x7               g114(.a(new_n161), .b(new_n189), .c(new_n185), .d(new_n193), .out0(new_n210));
  aoai13aa1n03x5               g115(.a(new_n210), .b(new_n125), .c(new_n111), .d(new_n119), .o1(new_n211));
  inv000aa1d42x5               g116(.a(new_n206), .o1(new_n212));
  xnrc02aa1n12x5               g117(.a(\b[17] ), .b(\a[18] ), .out0(new_n213));
  nano22aa1n09x5               g118(.a(new_n213), .b(new_n212), .c(new_n207), .out0(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  inv000aa1d42x5               g120(.a(\b[17] ), .o1(new_n216));
  oaoi03aa1n12x5               g121(.a(new_n205), .b(new_n216), .c(new_n206), .o1(new_n217));
  aoai13aa1n02x5               g122(.a(new_n217), .b(new_n215), .c(new_n211), .d(new_n202), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g124(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n09x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  nand22aa1n03x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  norp02aa1n02x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  nand42aa1n02x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(new_n225));
  aoi112aa1n02x5               g130(.a(new_n225), .b(new_n221), .c(new_n218), .d(new_n222), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n221), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n222), .b(new_n221), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n217), .o1(new_n229));
  aoai13aa1n03x5               g134(.a(new_n228), .b(new_n229), .c(new_n203), .d(new_n214), .o1(new_n230));
  aobi12aa1n03x5               g135(.a(new_n225), .b(new_n230), .c(new_n227), .out0(new_n231));
  nor002aa1n02x5               g136(.a(new_n231), .b(new_n226), .o1(\s[20] ));
  nona23aa1n12x5               g137(.a(new_n224), .b(new_n222), .c(new_n221), .d(new_n223), .out0(new_n233));
  inv040aa1n03x5               g138(.a(new_n233), .o1(new_n234));
  nand02aa1d06x5               g139(.a(new_n234), .b(new_n214), .o1(new_n235));
  tech160nm_fioai012aa1n02p5x5 g140(.a(new_n224), .b(new_n223), .c(new_n221), .o1(new_n236));
  oai012aa1n18x5               g141(.a(new_n236), .b(new_n233), .c(new_n217), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  aoai13aa1n02x5               g143(.a(new_n238), .b(new_n235), .c(new_n211), .d(new_n202), .o1(new_n239));
  xorb03aa1n02x5               g144(.a(new_n239), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n09x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  xnrc02aa1n12x5               g146(.a(\b[20] ), .b(\a[21] ), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  xnrc02aa1n12x5               g148(.a(\b[21] ), .b(\a[22] ), .out0(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  aoi112aa1n02x5               g150(.a(new_n241), .b(new_n245), .c(new_n239), .d(new_n243), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n241), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n235), .o1(new_n248));
  aoai13aa1n03x5               g153(.a(new_n243), .b(new_n237), .c(new_n203), .d(new_n248), .o1(new_n249));
  aoi012aa1n03x5               g154(.a(new_n244), .b(new_n249), .c(new_n247), .o1(new_n250));
  norp02aa1n03x5               g155(.a(new_n250), .b(new_n246), .o1(\s[22] ));
  nor042aa1n04x5               g156(.a(new_n244), .b(new_n242), .o1(new_n252));
  nand23aa1n06x5               g157(.a(new_n234), .b(new_n252), .c(new_n214), .o1(new_n253));
  oaoi03aa1n02x5               g158(.a(\a[22] ), .b(\b[21] ), .c(new_n247), .o1(new_n254));
  aoi012aa1d18x5               g159(.a(new_n254), .b(new_n237), .c(new_n252), .o1(new_n255));
  aoai13aa1n02x5               g160(.a(new_n255), .b(new_n253), .c(new_n211), .d(new_n202), .o1(new_n256));
  xorb03aa1n02x5               g161(.a(new_n256), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor022aa1n16x5               g162(.a(\b[22] ), .b(\a[23] ), .o1(new_n258));
  nand42aa1n02x5               g163(.a(\b[22] ), .b(\a[23] ), .o1(new_n259));
  nor002aa1n02x5               g164(.a(\b[23] ), .b(\a[24] ), .o1(new_n260));
  nand42aa1n03x5               g165(.a(\b[23] ), .b(\a[24] ), .o1(new_n261));
  norb02aa1n02x5               g166(.a(new_n261), .b(new_n260), .out0(new_n262));
  aoi112aa1n03x4               g167(.a(new_n258), .b(new_n262), .c(new_n256), .d(new_n259), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n258), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n253), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n255), .o1(new_n266));
  norb02aa1n02x5               g171(.a(new_n259), .b(new_n258), .out0(new_n267));
  aoai13aa1n03x5               g172(.a(new_n267), .b(new_n266), .c(new_n203), .d(new_n265), .o1(new_n268));
  aobi12aa1n03x5               g173(.a(new_n262), .b(new_n268), .c(new_n264), .out0(new_n269));
  nor002aa1n02x5               g174(.a(new_n269), .b(new_n263), .o1(\s[24] ));
  nona23aa1d18x5               g175(.a(new_n261), .b(new_n259), .c(new_n258), .d(new_n260), .out0(new_n271));
  inv000aa1d42x5               g176(.a(new_n271), .o1(new_n272));
  nano22aa1n02x4               g177(.a(new_n235), .b(new_n252), .c(new_n272), .out0(new_n273));
  inv000aa1n02x5               g178(.a(new_n273), .o1(new_n274));
  nona32aa1n09x5               g179(.a(new_n237), .b(new_n271), .c(new_n244), .d(new_n242), .out0(new_n275));
  nona22aa1n02x4               g180(.a(new_n261), .b(new_n260), .c(new_n258), .out0(new_n276));
  aboi22aa1n03x5               g181(.a(new_n271), .b(new_n254), .c(new_n261), .d(new_n276), .out0(new_n277));
  nanp02aa1n02x5               g182(.a(new_n275), .b(new_n277), .o1(new_n278));
  inv000aa1n02x5               g183(.a(new_n278), .o1(new_n279));
  aoai13aa1n02x5               g184(.a(new_n279), .b(new_n274), .c(new_n211), .d(new_n202), .o1(new_n280));
  xorb03aa1n02x5               g185(.a(new_n280), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g186(.a(\b[24] ), .b(\a[25] ), .o1(new_n282));
  xorc02aa1n02x5               g187(.a(\a[25] ), .b(\b[24] ), .out0(new_n283));
  xorc02aa1n02x5               g188(.a(\a[26] ), .b(\b[25] ), .out0(new_n284));
  aoi112aa1n03x4               g189(.a(new_n282), .b(new_n284), .c(new_n280), .d(new_n283), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n282), .o1(new_n286));
  aoai13aa1n03x5               g191(.a(new_n283), .b(new_n278), .c(new_n203), .d(new_n273), .o1(new_n287));
  aobi12aa1n03x5               g192(.a(new_n284), .b(new_n287), .c(new_n286), .out0(new_n288));
  nor002aa1n02x5               g193(.a(new_n288), .b(new_n285), .o1(\s[26] ));
  nano32aa1n03x7               g194(.a(new_n253), .b(new_n284), .c(new_n272), .d(new_n283), .out0(new_n290));
  nanp02aa1n02x5               g195(.a(new_n284), .b(new_n283), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[26] ), .b(\b[25] ), .c(new_n286), .carry(new_n292));
  aoai13aa1n09x5               g197(.a(new_n292), .b(new_n291), .c(new_n275), .d(new_n277), .o1(new_n293));
  xorc02aa1n02x5               g198(.a(\a[27] ), .b(\b[26] ), .out0(new_n294));
  aoai13aa1n06x5               g199(.a(new_n294), .b(new_n293), .c(new_n203), .d(new_n290), .o1(new_n295));
  aoi112aa1n02x5               g200(.a(new_n293), .b(new_n294), .c(new_n203), .d(new_n290), .o1(new_n296));
  norb02aa1n02x5               g201(.a(new_n295), .b(new_n296), .out0(\s[27] ));
  nor042aa1n09x5               g202(.a(\b[26] ), .b(\a[27] ), .o1(new_n298));
  xorc02aa1n02x5               g203(.a(\a[28] ), .b(\b[27] ), .out0(new_n299));
  nona22aa1n03x5               g204(.a(new_n295), .b(new_n299), .c(new_n298), .out0(new_n300));
  inv000aa1d42x5               g205(.a(new_n298), .o1(new_n301));
  inv000aa1d42x5               g206(.a(new_n299), .o1(new_n302));
  aoi012aa1n03x5               g207(.a(new_n302), .b(new_n295), .c(new_n301), .o1(new_n303));
  norb02aa1n03x4               g208(.a(new_n300), .b(new_n303), .out0(\s[28] ));
  tech160nm_fixorc02aa1n04x5   g209(.a(\a[29] ), .b(\b[28] ), .out0(new_n305));
  inv000aa1d42x5               g210(.a(new_n305), .o1(new_n306));
  inv000aa1d42x5               g211(.a(\a[27] ), .o1(new_n307));
  inv000aa1d42x5               g212(.a(\a[28] ), .o1(new_n308));
  xroi22aa1d04x5               g213(.a(new_n307), .b(\b[26] ), .c(new_n308), .d(\b[27] ), .out0(new_n309));
  aoai13aa1n06x5               g214(.a(new_n309), .b(new_n293), .c(new_n203), .d(new_n290), .o1(new_n310));
  inv000aa1d42x5               g215(.a(\b[27] ), .o1(new_n311));
  oaoi03aa1n09x5               g216(.a(new_n308), .b(new_n311), .c(new_n298), .o1(new_n312));
  aoi012aa1n03x5               g217(.a(new_n306), .b(new_n310), .c(new_n312), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n312), .o1(new_n314));
  nona22aa1n03x5               g219(.a(new_n310), .b(new_n314), .c(new_n305), .out0(new_n315));
  norb02aa1n03x4               g220(.a(new_n315), .b(new_n313), .out0(\s[29] ));
  xorb03aa1n02x5               g221(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb02aa1n02x5               g222(.a(new_n309), .b(new_n306), .out0(new_n318));
  aoai13aa1n06x5               g223(.a(new_n318), .b(new_n293), .c(new_n203), .d(new_n290), .o1(new_n319));
  oaoi03aa1n02x5               g224(.a(\a[29] ), .b(\b[28] ), .c(new_n312), .o1(new_n320));
  inv000aa1n03x5               g225(.a(new_n320), .o1(new_n321));
  xorc02aa1n12x5               g226(.a(\a[30] ), .b(\b[29] ), .out0(new_n322));
  inv000aa1d42x5               g227(.a(new_n322), .o1(new_n323));
  aoi012aa1n02x7               g228(.a(new_n323), .b(new_n319), .c(new_n321), .o1(new_n324));
  nona22aa1n03x5               g229(.a(new_n319), .b(new_n320), .c(new_n322), .out0(new_n325));
  norb02aa1n02x7               g230(.a(new_n325), .b(new_n324), .out0(\s[30] ));
  and003aa1n02x5               g231(.a(new_n309), .b(new_n305), .c(new_n322), .o(new_n327));
  aoai13aa1n06x5               g232(.a(new_n327), .b(new_n293), .c(new_n203), .d(new_n290), .o1(new_n328));
  oao003aa1n02x5               g233(.a(\a[30] ), .b(\b[29] ), .c(new_n321), .carry(new_n329));
  inv000aa1n02x5               g234(.a(new_n329), .o1(new_n330));
  xorc02aa1n02x5               g235(.a(\a[31] ), .b(\b[30] ), .out0(new_n331));
  nona22aa1n03x5               g236(.a(new_n328), .b(new_n330), .c(new_n331), .out0(new_n332));
  inv000aa1d42x5               g237(.a(new_n331), .o1(new_n333));
  aoi012aa1n03x5               g238(.a(new_n333), .b(new_n328), .c(new_n329), .o1(new_n334));
  norb02aa1n03x4               g239(.a(new_n332), .b(new_n334), .out0(\s[31] ));
  xnbna2aa1n03x5               g240(.a(new_n109), .b(new_n103), .c(new_n104), .out0(\s[3] ));
  oaoi03aa1n02x5               g241(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n337));
  xorb03aa1n02x5               g242(.a(new_n337), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g243(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g244(.a(\a[5] ), .b(\b[4] ), .o(new_n340));
  nona22aa1n02x4               g245(.a(new_n139), .b(new_n118), .c(new_n100), .out0(new_n341));
  xobna2aa1n03x5               g246(.a(new_n117), .b(new_n341), .c(new_n340), .out0(\s[6] ));
  aoi012aa1n02x5               g247(.a(new_n122), .b(new_n120), .c(new_n121), .o1(new_n343));
  aoi022aa1n02x5               g248(.a(new_n341), .b(new_n343), .c(\a[6] ), .d(\b[5] ), .o1(new_n344));
  xorb03aa1n02x5               g249(.a(new_n344), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g250(.a(new_n115), .b(new_n344), .c(new_n114), .o1(new_n346));
  xnrb03aa1n02x5               g251(.a(new_n346), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g252(.a(new_n126), .b(new_n141), .c(new_n142), .out0(\s[9] ));
endmodule


