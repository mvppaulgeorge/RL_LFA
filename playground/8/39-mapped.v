// Benchmark "adder" written by ABC on Wed Jul 17 16:23:50 2024

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
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n195,
    new_n196, new_n197, new_n198, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n337, new_n338,
    new_n341, new_n342, new_n344, new_n345, new_n346, new_n348;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n09x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor042aa1n04x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand02aa1d12x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nand02aa1d12x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  aoi012aa1n12x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  nor042aa1n03x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nand02aa1n03x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norb02aa1n03x5               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  xorc02aa1n12x5               g009(.a(\a[3] ), .b(\b[2] ), .out0(new_n105));
  nanb03aa1n09x5               g010(.a(new_n101), .b(new_n105), .c(new_n104), .out0(new_n106));
  inv040aa1d32x5               g011(.a(\a[3] ), .o1(new_n107));
  inv040aa1d28x5               g012(.a(\b[2] ), .o1(new_n108));
  nand02aa1d16x5               g013(.a(new_n108), .b(new_n107), .o1(new_n109));
  oaoi03aa1n12x5               g014(.a(\a[4] ), .b(\b[3] ), .c(new_n109), .o1(new_n110));
  inv030aa1n02x5               g015(.a(new_n110), .o1(new_n111));
  tech160nm_fixnrc02aa1n04x5   g016(.a(\b[4] ), .b(\a[5] ), .out0(new_n112));
  nor002aa1d32x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nand02aa1d16x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nanb02aa1n02x5               g019(.a(new_n113), .b(new_n114), .out0(new_n115));
  norp02aa1n04x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nand22aa1n04x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nor002aa1n02x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  nano23aa1n02x4               g024(.a(new_n116), .b(new_n118), .c(new_n119), .d(new_n117), .out0(new_n120));
  nona22aa1n03x5               g025(.a(new_n120), .b(new_n112), .c(new_n115), .out0(new_n121));
  inv000aa1d42x5               g026(.a(\a[8] ), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\b[7] ), .o1(new_n123));
  inv000aa1d42x5               g028(.a(\a[7] ), .o1(new_n124));
  inv040aa1d32x5               g029(.a(\a[5] ), .o1(new_n125));
  inv000aa1n36x5               g030(.a(\b[4] ), .o1(new_n126));
  aoai13aa1n12x5               g031(.a(new_n114), .b(new_n113), .c(new_n125), .d(new_n126), .o1(new_n127));
  oaib12aa1n09x5               g032(.a(new_n127), .b(\b[6] ), .c(new_n124), .out0(new_n128));
  aoi022aa1n02x5               g033(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n129));
  aoi022aa1n12x5               g034(.a(new_n128), .b(new_n129), .c(new_n123), .d(new_n122), .o1(new_n130));
  aoai13aa1n12x5               g035(.a(new_n130), .b(new_n121), .c(new_n106), .d(new_n111), .o1(new_n131));
  nand42aa1n02x5               g036(.a(\b[8] ), .b(\a[9] ), .o1(new_n132));
  aoi012aa1n02x5               g037(.a(new_n97), .b(new_n131), .c(new_n132), .o1(new_n133));
  xnrb03aa1n02x5               g038(.a(new_n133), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nanb02aa1n02x5               g039(.a(new_n102), .b(new_n103), .out0(new_n135));
  nanp02aa1n02x5               g040(.a(\b[2] ), .b(\a[3] ), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(new_n109), .b(new_n136), .o1(new_n137));
  nor003aa1n02x5               g042(.a(new_n101), .b(new_n135), .c(new_n137), .o1(new_n138));
  nona23aa1n03x5               g043(.a(new_n119), .b(new_n117), .c(new_n116), .d(new_n118), .out0(new_n139));
  nor003aa1n02x5               g044(.a(new_n139), .b(new_n115), .c(new_n112), .o1(new_n140));
  tech160nm_fioai012aa1n04x5   g045(.a(new_n140), .b(new_n138), .c(new_n110), .o1(new_n141));
  nor042aa1n04x5               g046(.a(\b[9] ), .b(\a[10] ), .o1(new_n142));
  nand22aa1n06x5               g047(.a(\b[9] ), .b(\a[10] ), .o1(new_n143));
  nona23aa1n09x5               g048(.a(new_n132), .b(new_n143), .c(new_n142), .d(new_n97), .out0(new_n144));
  aoi012aa1n06x5               g049(.a(new_n142), .b(new_n97), .c(new_n143), .o1(new_n145));
  aoai13aa1n02x5               g050(.a(new_n145), .b(new_n144), .c(new_n141), .d(new_n130), .o1(new_n146));
  xorb03aa1n02x5               g051(.a(new_n146), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n08x5               g052(.a(\b[10] ), .b(\a[11] ), .o1(new_n148));
  nand22aa1n04x5               g053(.a(\b[10] ), .b(\a[11] ), .o1(new_n149));
  aoi012aa1n02x5               g054(.a(new_n148), .b(new_n146), .c(new_n149), .o1(new_n150));
  xnrb03aa1n02x5               g055(.a(new_n150), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor022aa1n08x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nand22aa1n09x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  nona23aa1n09x5               g058(.a(new_n153), .b(new_n149), .c(new_n148), .d(new_n152), .out0(new_n154));
  nor042aa1n06x5               g059(.a(new_n154), .b(new_n144), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  inv000aa1n02x5               g061(.a(new_n145), .o1(new_n157));
  nano23aa1n03x5               g062(.a(new_n148), .b(new_n152), .c(new_n153), .d(new_n149), .out0(new_n158));
  aoi012aa1n02x7               g063(.a(new_n152), .b(new_n148), .c(new_n153), .o1(new_n159));
  aobi12aa1n02x5               g064(.a(new_n159), .b(new_n158), .c(new_n157), .out0(new_n160));
  aoai13aa1n03x5               g065(.a(new_n160), .b(new_n156), .c(new_n141), .d(new_n130), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n12x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  nand02aa1n04x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  aoi012aa1n02x5               g069(.a(new_n163), .b(new_n161), .c(new_n164), .o1(new_n165));
  xnrb03aa1n03x5               g070(.a(new_n165), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  tech160nm_fioai012aa1n05x5   g071(.a(new_n159), .b(new_n154), .c(new_n145), .o1(new_n167));
  nor042aa1n06x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nand22aa1n09x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nona23aa1n09x5               g074(.a(new_n169), .b(new_n164), .c(new_n163), .d(new_n168), .out0(new_n170));
  inv040aa1n02x5               g075(.a(new_n170), .o1(new_n171));
  aoai13aa1n06x5               g076(.a(new_n171), .b(new_n167), .c(new_n131), .d(new_n155), .o1(new_n172));
  aoi012aa1d24x5               g077(.a(new_n168), .b(new_n163), .c(new_n169), .o1(new_n173));
  nor042aa1n04x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nanp02aa1n03x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  xnbna2aa1n03x5               g081(.a(new_n176), .b(new_n172), .c(new_n173), .out0(\s[15] ));
  nanp02aa1n02x5               g082(.a(new_n172), .b(new_n173), .o1(new_n178));
  tech160nm_finor002aa1n05x5   g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nand42aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  nanb02aa1n02x5               g085(.a(new_n179), .b(new_n180), .out0(new_n181));
  aoai13aa1n03x5               g086(.a(new_n181), .b(new_n174), .c(new_n178), .d(new_n176), .o1(new_n182));
  inv000aa1d42x5               g087(.a(new_n173), .o1(new_n183));
  aoai13aa1n02x5               g088(.a(new_n176), .b(new_n183), .c(new_n161), .d(new_n171), .o1(new_n184));
  nona22aa1n02x5               g089(.a(new_n184), .b(new_n181), .c(new_n174), .out0(new_n185));
  nanp02aa1n02x5               g090(.a(new_n182), .b(new_n185), .o1(\s[16] ));
  nona23aa1n09x5               g091(.a(new_n180), .b(new_n175), .c(new_n174), .d(new_n179), .out0(new_n187));
  nor042aa1n06x5               g092(.a(new_n187), .b(new_n170), .o1(new_n188));
  nand02aa1d04x5               g093(.a(new_n155), .b(new_n188), .o1(new_n189));
  tech160nm_fiaoi012aa1n03p5x5 g094(.a(new_n179), .b(new_n174), .c(new_n180), .o1(new_n190));
  tech160nm_fioai012aa1n05x5   g095(.a(new_n190), .b(new_n187), .c(new_n173), .o1(new_n191));
  aoi012aa1n06x5               g096(.a(new_n191), .b(new_n167), .c(new_n188), .o1(new_n192));
  aoai13aa1n06x5               g097(.a(new_n192), .b(new_n189), .c(new_n141), .d(new_n130), .o1(new_n193));
  xorb03aa1n02x5               g098(.a(new_n193), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g099(.a(\a[18] ), .o1(new_n195));
  inv000aa1d42x5               g100(.a(\a[17] ), .o1(new_n196));
  inv000aa1d42x5               g101(.a(\b[16] ), .o1(new_n197));
  oaoi03aa1n03x5               g102(.a(new_n196), .b(new_n197), .c(new_n193), .o1(new_n198));
  xorb03aa1n02x5               g103(.a(new_n198), .b(\b[17] ), .c(new_n195), .out0(\s[18] ));
  inv040aa1n03x5               g104(.a(new_n189), .o1(new_n200));
  nanb02aa1n02x5               g105(.a(new_n187), .b(new_n171), .out0(new_n201));
  oabi12aa1n06x5               g106(.a(new_n191), .b(new_n201), .c(new_n160), .out0(new_n202));
  xroi22aa1d06x4               g107(.a(new_n196), .b(\b[16] ), .c(new_n195), .d(\b[17] ), .out0(new_n203));
  aoai13aa1n06x5               g108(.a(new_n203), .b(new_n202), .c(new_n131), .d(new_n200), .o1(new_n204));
  nor042aa1n02x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  aoi112aa1n09x5               g110(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n206));
  norp02aa1n02x5               g111(.a(new_n206), .b(new_n205), .o1(new_n207));
  nor042aa1n06x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nand02aa1n06x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  norb02aa1n06x5               g114(.a(new_n209), .b(new_n208), .out0(new_n210));
  xnbna2aa1n03x5               g115(.a(new_n210), .b(new_n204), .c(new_n207), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n02x5               g117(.a(new_n204), .b(new_n207), .o1(new_n213));
  nor042aa1d18x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nand42aa1d28x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  norb02aa1n15x5               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  aoai13aa1n02x5               g122(.a(new_n217), .b(new_n208), .c(new_n213), .d(new_n209), .o1(new_n218));
  inv000aa1n02x5               g123(.a(new_n207), .o1(new_n219));
  aoai13aa1n02x5               g124(.a(new_n210), .b(new_n219), .c(new_n193), .d(new_n203), .o1(new_n220));
  nona22aa1n02x4               g125(.a(new_n220), .b(new_n217), .c(new_n208), .out0(new_n221));
  nanp02aa1n03x5               g126(.a(new_n218), .b(new_n221), .o1(\s[20] ));
  nanp02aa1n06x5               g127(.a(new_n131), .b(new_n200), .o1(new_n223));
  nano23aa1n09x5               g128(.a(new_n208), .b(new_n214), .c(new_n215), .d(new_n209), .out0(new_n224));
  nand02aa1d04x5               g129(.a(new_n203), .b(new_n224), .o1(new_n225));
  oai112aa1n06x5               g130(.a(new_n210), .b(new_n216), .c(new_n206), .d(new_n205), .o1(new_n226));
  tech160nm_fiaoi012aa1n05x5   g131(.a(new_n214), .b(new_n208), .c(new_n215), .o1(new_n227));
  nand22aa1n12x5               g132(.a(new_n226), .b(new_n227), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n225), .c(new_n223), .d(new_n192), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  xorc02aa1n02x5               g137(.a(\a[21] ), .b(\b[20] ), .out0(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[21] ), .b(\a[22] ), .out0(new_n234));
  aoai13aa1n03x5               g139(.a(new_n234), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n225), .o1(new_n236));
  aoai13aa1n03x5               g141(.a(new_n233), .b(new_n228), .c(new_n193), .d(new_n236), .o1(new_n237));
  nona22aa1n02x4               g142(.a(new_n237), .b(new_n234), .c(new_n232), .out0(new_n238));
  nanp02aa1n03x5               g143(.a(new_n235), .b(new_n238), .o1(\s[22] ));
  xnrc02aa1n12x5               g144(.a(\b[20] ), .b(\a[21] ), .out0(new_n240));
  nor042aa1n06x5               g145(.a(new_n234), .b(new_n240), .o1(new_n241));
  nand23aa1d12x5               g146(.a(new_n203), .b(new_n241), .c(new_n224), .o1(new_n242));
  norp02aa1n02x5               g147(.a(\b[21] ), .b(\a[22] ), .o1(new_n243));
  nanp02aa1n02x5               g148(.a(\b[21] ), .b(\a[22] ), .o1(new_n244));
  aoi012aa1n12x5               g149(.a(new_n243), .b(new_n232), .c(new_n244), .o1(new_n245));
  aobi12aa1n18x5               g150(.a(new_n245), .b(new_n228), .c(new_n241), .out0(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n242), .c(new_n223), .d(new_n192), .o1(new_n247));
  xorb03aa1n02x5               g152(.a(new_n247), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n06x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  nanp02aa1n12x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  nanb02aa1n12x5               g155(.a(new_n249), .b(new_n250), .out0(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  nor042aa1n12x5               g157(.a(\b[23] ), .b(\a[24] ), .o1(new_n253));
  nand22aa1n03x5               g158(.a(\b[23] ), .b(\a[24] ), .o1(new_n254));
  nanb02aa1n03x5               g159(.a(new_n253), .b(new_n254), .out0(new_n255));
  aoai13aa1n03x5               g160(.a(new_n255), .b(new_n249), .c(new_n247), .d(new_n252), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n242), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n246), .o1(new_n258));
  aoai13aa1n03x5               g163(.a(new_n252), .b(new_n258), .c(new_n193), .d(new_n257), .o1(new_n259));
  nona22aa1n02x4               g164(.a(new_n259), .b(new_n255), .c(new_n249), .out0(new_n260));
  nanp02aa1n03x5               g165(.a(new_n256), .b(new_n260), .o1(\s[24] ));
  nona23aa1d18x5               g166(.a(new_n254), .b(new_n250), .c(new_n249), .d(new_n253), .out0(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  nano22aa1n03x7               g168(.a(new_n225), .b(new_n241), .c(new_n263), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n202), .c(new_n131), .d(new_n200), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n253), .o1(new_n266));
  nanp02aa1n02x5               g171(.a(new_n249), .b(new_n254), .o1(new_n267));
  oai112aa1n06x5               g172(.a(new_n267), .b(new_n266), .c(new_n262), .d(new_n245), .o1(new_n268));
  aoi013aa1n09x5               g173(.a(new_n268), .b(new_n228), .c(new_n241), .d(new_n263), .o1(new_n269));
  xorc02aa1n12x5               g174(.a(\a[25] ), .b(\b[24] ), .out0(new_n270));
  xnbna2aa1n03x5               g175(.a(new_n270), .b(new_n265), .c(new_n269), .out0(\s[25] ));
  nand42aa1n03x5               g176(.a(new_n265), .b(new_n269), .o1(new_n272));
  norp02aa1n02x5               g177(.a(\b[24] ), .b(\a[25] ), .o1(new_n273));
  xnrc02aa1n12x5               g178(.a(\b[25] ), .b(\a[26] ), .out0(new_n274));
  aoai13aa1n03x5               g179(.a(new_n274), .b(new_n273), .c(new_n272), .d(new_n270), .o1(new_n275));
  inv000aa1n02x5               g180(.a(new_n269), .o1(new_n276));
  aoai13aa1n02x5               g181(.a(new_n270), .b(new_n276), .c(new_n193), .d(new_n264), .o1(new_n277));
  nona22aa1n02x4               g182(.a(new_n277), .b(new_n274), .c(new_n273), .out0(new_n278));
  nanp02aa1n03x5               g183(.a(new_n275), .b(new_n278), .o1(\s[26] ));
  norb02aa1n06x5               g184(.a(new_n270), .b(new_n274), .out0(new_n280));
  nano22aa1d15x5               g185(.a(new_n242), .b(new_n263), .c(new_n280), .out0(new_n281));
  aoai13aa1n06x5               g186(.a(new_n281), .b(new_n202), .c(new_n131), .d(new_n200), .o1(new_n282));
  xorc02aa1n02x5               g187(.a(\a[22] ), .b(\b[21] ), .out0(new_n283));
  nanp02aa1n02x5               g188(.a(new_n283), .b(new_n233), .o1(new_n284));
  aoi112aa1n02x7               g189(.a(new_n262), .b(new_n284), .c(new_n226), .d(new_n227), .o1(new_n285));
  inv000aa1d42x5               g190(.a(\a[26] ), .o1(new_n286));
  inv000aa1d42x5               g191(.a(\b[25] ), .o1(new_n287));
  oaoi03aa1n02x5               g192(.a(new_n286), .b(new_n287), .c(new_n273), .o1(new_n288));
  inv000aa1n02x5               g193(.a(new_n288), .o1(new_n289));
  oaoi13aa1n06x5               g194(.a(new_n289), .b(new_n280), .c(new_n285), .d(new_n268), .o1(new_n290));
  tech160nm_fixorc02aa1n03p5x5 g195(.a(\a[27] ), .b(\b[26] ), .out0(new_n291));
  xnbna2aa1n03x5               g196(.a(new_n291), .b(new_n282), .c(new_n290), .out0(\s[27] ));
  nand42aa1n04x5               g197(.a(new_n282), .b(new_n290), .o1(new_n293));
  norp02aa1n02x5               g198(.a(\b[26] ), .b(\a[27] ), .o1(new_n294));
  norp02aa1n02x5               g199(.a(\b[27] ), .b(\a[28] ), .o1(new_n295));
  nand42aa1n08x5               g200(.a(\b[27] ), .b(\a[28] ), .o1(new_n296));
  norb02aa1n09x5               g201(.a(new_n296), .b(new_n295), .out0(new_n297));
  inv000aa1d42x5               g202(.a(new_n297), .o1(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n294), .c(new_n293), .d(new_n291), .o1(new_n299));
  norp03aa1n02x5               g204(.a(new_n245), .b(new_n251), .c(new_n255), .o1(new_n300));
  nano22aa1n02x4               g205(.a(new_n300), .b(new_n266), .c(new_n267), .out0(new_n301));
  nona32aa1n03x5               g206(.a(new_n228), .b(new_n262), .c(new_n234), .d(new_n240), .out0(new_n302));
  inv000aa1d42x5               g207(.a(new_n280), .o1(new_n303));
  aoai13aa1n06x5               g208(.a(new_n288), .b(new_n303), .c(new_n302), .d(new_n301), .o1(new_n304));
  aoai13aa1n03x5               g209(.a(new_n291), .b(new_n304), .c(new_n193), .d(new_n281), .o1(new_n305));
  nona22aa1n02x5               g210(.a(new_n305), .b(new_n298), .c(new_n294), .out0(new_n306));
  nanp02aa1n03x5               g211(.a(new_n299), .b(new_n306), .o1(\s[28] ));
  norb02aa1n09x5               g212(.a(new_n291), .b(new_n298), .out0(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n304), .c(new_n193), .d(new_n281), .o1(new_n309));
  aoi012aa1n02x5               g214(.a(new_n295), .b(new_n294), .c(new_n296), .o1(new_n310));
  xnrc02aa1n02x5               g215(.a(\b[28] ), .b(\a[29] ), .out0(new_n311));
  tech160nm_fiaoi012aa1n02p5x5 g216(.a(new_n311), .b(new_n309), .c(new_n310), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n308), .o1(new_n313));
  tech160nm_fiaoi012aa1n02p5x5 g218(.a(new_n313), .b(new_n282), .c(new_n290), .o1(new_n314));
  nano22aa1n03x5               g219(.a(new_n314), .b(new_n310), .c(new_n311), .out0(new_n315));
  nor002aa1n02x5               g220(.a(new_n312), .b(new_n315), .o1(\s[29] ));
  xorb03aa1n02x5               g221(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n09x5               g222(.a(new_n311), .b(new_n291), .c(new_n297), .out0(new_n318));
  aoai13aa1n03x5               g223(.a(new_n318), .b(new_n304), .c(new_n193), .d(new_n281), .o1(new_n319));
  oao003aa1n02x5               g224(.a(\a[29] ), .b(\b[28] ), .c(new_n310), .carry(new_n320));
  xnrc02aa1n02x5               g225(.a(\b[29] ), .b(\a[30] ), .out0(new_n321));
  tech160nm_fiaoi012aa1n02p5x5 g226(.a(new_n321), .b(new_n319), .c(new_n320), .o1(new_n322));
  inv000aa1d42x5               g227(.a(new_n318), .o1(new_n323));
  tech160nm_fiaoi012aa1n02p5x5 g228(.a(new_n323), .b(new_n282), .c(new_n290), .o1(new_n324));
  nano22aa1n03x5               g229(.a(new_n324), .b(new_n320), .c(new_n321), .out0(new_n325));
  norp02aa1n03x5               g230(.a(new_n322), .b(new_n325), .o1(\s[30] ));
  nano23aa1n06x5               g231(.a(new_n321), .b(new_n311), .c(new_n291), .d(new_n297), .out0(new_n327));
  aoai13aa1n03x5               g232(.a(new_n327), .b(new_n304), .c(new_n193), .d(new_n281), .o1(new_n328));
  oao003aa1n02x5               g233(.a(\a[30] ), .b(\b[29] ), .c(new_n320), .carry(new_n329));
  xnrc02aa1n02x5               g234(.a(\b[30] ), .b(\a[31] ), .out0(new_n330));
  tech160nm_fiaoi012aa1n02p5x5 g235(.a(new_n330), .b(new_n328), .c(new_n329), .o1(new_n331));
  inv000aa1n02x5               g236(.a(new_n327), .o1(new_n332));
  tech160nm_fiaoi012aa1n02p5x5 g237(.a(new_n332), .b(new_n282), .c(new_n290), .o1(new_n333));
  nano22aa1n03x5               g238(.a(new_n333), .b(new_n329), .c(new_n330), .out0(new_n334));
  norp02aa1n03x5               g239(.a(new_n331), .b(new_n334), .o1(\s[31] ));
  xnbna2aa1n03x5               g240(.a(new_n101), .b(new_n136), .c(new_n109), .out0(\s[3] ));
  oai112aa1n02x5               g241(.a(new_n109), .b(new_n135), .c(new_n101), .d(new_n137), .o1(new_n337));
  aoi012aa1n02x5               g242(.a(new_n102), .b(new_n106), .c(new_n111), .o1(new_n338));
  norb02aa1n02x5               g243(.a(new_n337), .b(new_n338), .out0(\s[4] ));
  xobna2aa1n03x5               g244(.a(new_n112), .b(new_n106), .c(new_n111), .out0(\s[5] ));
  nanp02aa1n02x5               g245(.a(new_n126), .b(new_n125), .o1(new_n341));
  aoai13aa1n06x5               g246(.a(new_n341), .b(new_n112), .c(new_n106), .d(new_n111), .o1(new_n342));
  xorb03aa1n02x5               g247(.a(new_n342), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g248(.a(new_n117), .b(new_n116), .out0(new_n344));
  oaoi13aa1n02x5               g249(.a(new_n344), .b(new_n114), .c(new_n342), .d(new_n113), .o1(new_n345));
  oai112aa1n02x5               g250(.a(new_n344), .b(new_n114), .c(new_n342), .d(new_n113), .o1(new_n346));
  norb02aa1n02x5               g251(.a(new_n346), .b(new_n345), .out0(\s[7] ));
  oaib12aa1n02x5               g252(.a(new_n346), .b(\b[6] ), .c(new_n124), .out0(new_n348));
  xorb03aa1n02x5               g253(.a(new_n348), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g254(.a(new_n131), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


