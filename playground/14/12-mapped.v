// Benchmark "adder" written by ABC on Wed Jul 17 19:12:48 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n168, new_n169, new_n170,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n347, new_n348, new_n351, new_n353, new_n355;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d30x5               g001(.a(\a[9] ), .o1(new_n97));
  inv040aa1d30x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  and002aa1n24x5               g004(.a(\b[3] ), .b(\a[4] ), .o(new_n100));
  inv000aa1d42x5               g005(.a(\a[3] ), .o1(new_n101));
  inv040aa1d32x5               g006(.a(\b[2] ), .o1(new_n102));
  nor042aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  aoi112aa1n06x5               g008(.a(new_n100), .b(new_n103), .c(new_n101), .d(new_n102), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(new_n102), .b(new_n101), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(new_n105), .b(new_n106), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nor042aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  nand02aa1n04x5               g014(.a(\b[0] ), .b(\a[1] ), .o1(new_n110));
  oai012aa1n06x5               g015(.a(new_n108), .b(new_n109), .c(new_n110), .o1(new_n111));
  oaoi13aa1n09x5               g016(.a(new_n100), .b(new_n104), .c(new_n111), .d(new_n107), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\a[6] ), .o1(new_n114));
  inv000aa1d42x5               g019(.a(\b[5] ), .o1(new_n115));
  nand42aa1n03x5               g020(.a(new_n115), .b(new_n114), .o1(new_n116));
  nand42aa1n03x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  oai112aa1n03x5               g022(.a(new_n116), .b(new_n117), .c(\b[4] ), .d(\a[5] ), .o1(new_n118));
  nand02aa1n03x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nor022aa1n16x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  inv000aa1n02x5               g025(.a(new_n120), .o1(new_n121));
  nand42aa1n04x5               g026(.a(\b[7] ), .b(\a[8] ), .o1(new_n122));
  oai112aa1n02x5               g027(.a(new_n121), .b(new_n122), .c(\b[6] ), .d(\a[7] ), .o1(new_n123));
  nano23aa1n03x7               g028(.a(new_n123), .b(new_n118), .c(new_n113), .d(new_n119), .out0(new_n124));
  norp02aa1n04x5               g029(.a(\b[6] ), .b(\a[7] ), .o1(new_n125));
  nona23aa1n06x5               g030(.a(new_n122), .b(new_n119), .c(new_n125), .d(new_n120), .out0(new_n126));
  oai012aa1n02x7               g031(.a(new_n122), .b(new_n125), .c(new_n120), .o1(new_n127));
  nor002aa1n02x5               g032(.a(\b[4] ), .b(\a[5] ), .o1(new_n128));
  oaoi03aa1n03x5               g033(.a(new_n114), .b(new_n115), .c(new_n128), .o1(new_n129));
  oai012aa1n18x5               g034(.a(new_n127), .b(new_n126), .c(new_n129), .o1(new_n130));
  xorc02aa1n02x5               g035(.a(\a[9] ), .b(\b[8] ), .out0(new_n131));
  aoai13aa1n04x5               g036(.a(new_n131), .b(new_n130), .c(new_n112), .d(new_n124), .o1(new_n132));
  nor002aa1d32x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nand42aa1d28x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n132), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g041(.a(new_n133), .o1(new_n137));
  inv000aa1n02x5               g042(.a(new_n134), .o1(new_n138));
  nor042aa1n09x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nand02aa1n03x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  inv000aa1n02x5               g046(.a(new_n141), .o1(new_n142));
  aoi113aa1n02x5               g047(.a(new_n142), .b(new_n138), .c(new_n132), .d(new_n137), .e(new_n99), .o1(new_n143));
  nanp02aa1n03x5               g048(.a(new_n132), .b(new_n99), .o1(new_n144));
  aoai13aa1n12x5               g049(.a(new_n134), .b(new_n133), .c(new_n97), .d(new_n98), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  aoi112aa1n02x5               g051(.a(new_n146), .b(new_n141), .c(new_n144), .d(new_n135), .o1(new_n147));
  norp02aa1n02x5               g052(.a(new_n147), .b(new_n143), .o1(\s[11] ));
  inv000aa1d42x5               g053(.a(new_n139), .o1(new_n149));
  nor002aa1n04x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  nand02aa1n03x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  norb02aa1n02x5               g056(.a(new_n151), .b(new_n150), .out0(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  nano22aa1n02x4               g058(.a(new_n143), .b(new_n149), .c(new_n153), .out0(new_n154));
  aoai13aa1n03x5               g059(.a(new_n141), .b(new_n146), .c(new_n144), .d(new_n135), .o1(new_n155));
  tech160nm_fiaoi012aa1n03p5x5 g060(.a(new_n153), .b(new_n155), .c(new_n149), .o1(new_n156));
  nor002aa1n02x5               g061(.a(new_n156), .b(new_n154), .o1(\s[12] ));
  and002aa1n06x5               g062(.a(\b[8] ), .b(\a[9] ), .o(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  nona23aa1n09x5               g064(.a(new_n151), .b(new_n140), .c(new_n139), .d(new_n150), .out0(new_n160));
  nano32aa1n02x4               g065(.a(new_n160), .b(new_n135), .c(new_n159), .d(new_n99), .out0(new_n161));
  aoai13aa1n03x5               g066(.a(new_n161), .b(new_n130), .c(new_n112), .d(new_n124), .o1(new_n162));
  tech160nm_fioai012aa1n03p5x5 g067(.a(new_n151), .b(new_n150), .c(new_n139), .o1(new_n163));
  oai012aa1d24x5               g068(.a(new_n163), .b(new_n160), .c(new_n145), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(new_n162), .b(new_n165), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g072(.a(\a[13] ), .o1(new_n168));
  inv000aa1d42x5               g073(.a(\b[12] ), .o1(new_n169));
  oaoi03aa1n02x5               g074(.a(new_n168), .b(new_n169), .c(new_n166), .o1(new_n170));
  xnrb03aa1n02x5               g075(.a(new_n170), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n08x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nand42aa1n02x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n172), .c(new_n168), .d(new_n169), .o1(new_n174));
  norp02aa1n02x5               g079(.a(\b[12] ), .b(\a[13] ), .o1(new_n175));
  nand22aa1n03x5               g080(.a(\b[12] ), .b(\a[13] ), .o1(new_n176));
  nona23aa1n02x4               g081(.a(new_n173), .b(new_n176), .c(new_n175), .d(new_n172), .out0(new_n177));
  aoai13aa1n06x5               g082(.a(new_n174), .b(new_n177), .c(new_n162), .d(new_n165), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  nand22aa1n06x5               g085(.a(\b[14] ), .b(\a[15] ), .o1(new_n181));
  nor002aa1d32x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  nand42aa1n02x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nanb02aa1n02x5               g088(.a(new_n182), .b(new_n183), .out0(new_n184));
  inv000aa1d42x5               g089(.a(new_n184), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(new_n185), .b(new_n180), .c(new_n178), .d(new_n181), .o1(new_n186));
  aoai13aa1n02x5               g091(.a(new_n185), .b(new_n180), .c(new_n178), .d(new_n181), .o1(new_n187));
  norb02aa1n03x4               g092(.a(new_n187), .b(new_n186), .out0(\s[16] ));
  aoi112aa1n02x5               g093(.a(new_n138), .b(new_n133), .c(new_n97), .d(new_n98), .o1(new_n189));
  nano23aa1n06x5               g094(.a(new_n139), .b(new_n150), .c(new_n151), .d(new_n140), .out0(new_n190));
  nano23aa1n02x4               g095(.a(new_n175), .b(new_n172), .c(new_n173), .d(new_n176), .out0(new_n191));
  nano23aa1n02x5               g096(.a(new_n180), .b(new_n182), .c(new_n183), .d(new_n181), .out0(new_n192));
  nand02aa1n02x5               g097(.a(new_n192), .b(new_n191), .o1(new_n193));
  nano32aa1n03x7               g098(.a(new_n193), .b(new_n190), .c(new_n189), .d(new_n159), .out0(new_n194));
  aoai13aa1n12x5               g099(.a(new_n194), .b(new_n130), .c(new_n112), .d(new_n124), .o1(new_n195));
  nona23aa1n02x4               g100(.a(new_n183), .b(new_n181), .c(new_n180), .d(new_n182), .out0(new_n196));
  norp02aa1n04x5               g101(.a(new_n196), .b(new_n177), .o1(new_n197));
  oa0012aa1n02x5               g102(.a(new_n183), .b(new_n182), .c(new_n180), .o(new_n198));
  nor022aa1n04x5               g103(.a(new_n196), .b(new_n174), .o1(new_n199));
  aoi112aa1n09x5               g104(.a(new_n199), .b(new_n198), .c(new_n164), .d(new_n197), .o1(new_n200));
  nand02aa1d06x5               g105(.a(new_n195), .b(new_n200), .o1(new_n201));
  xorb03aa1n02x5               g106(.a(new_n201), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g107(.a(\a[17] ), .o1(new_n203));
  inv000aa1d42x5               g108(.a(\b[16] ), .o1(new_n204));
  oaoi03aa1n03x5               g109(.a(new_n203), .b(new_n204), .c(new_n201), .o1(new_n205));
  xnrb03aa1n03x5               g110(.a(new_n205), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nanp02aa1n02x5               g111(.a(new_n204), .b(new_n203), .o1(new_n207));
  nanp02aa1n02x5               g112(.a(\b[16] ), .b(\a[17] ), .o1(new_n208));
  nor022aa1n08x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  nand42aa1n08x5               g114(.a(\b[17] ), .b(\a[18] ), .o1(new_n210));
  nanb02aa1n03x5               g115(.a(new_n209), .b(new_n210), .out0(new_n211));
  nano22aa1n09x5               g116(.a(new_n211), .b(new_n207), .c(new_n208), .out0(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  aoai13aa1n12x5               g118(.a(new_n210), .b(new_n209), .c(new_n203), .d(new_n204), .o1(new_n214));
  aoai13aa1n06x5               g119(.a(new_n214), .b(new_n213), .c(new_n195), .d(new_n200), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g121(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  nand42aa1n08x5               g123(.a(\b[18] ), .b(\a[19] ), .o1(new_n219));
  nor042aa1n02x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  nand42aa1n03x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nanb02aa1n02x5               g126(.a(new_n220), .b(new_n221), .out0(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  aoi112aa1n02x7               g128(.a(new_n218), .b(new_n223), .c(new_n215), .d(new_n219), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n218), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n219), .b(new_n218), .out0(new_n226));
  nanp02aa1n03x5               g131(.a(new_n215), .b(new_n226), .o1(new_n227));
  aoi012aa1n03x5               g132(.a(new_n222), .b(new_n227), .c(new_n225), .o1(new_n228));
  nor002aa1n02x5               g133(.a(new_n228), .b(new_n224), .o1(\s[20] ));
  nano23aa1n03x5               g134(.a(new_n218), .b(new_n220), .c(new_n221), .d(new_n219), .out0(new_n230));
  nanp02aa1n02x5               g135(.a(new_n212), .b(new_n230), .o1(new_n231));
  inv000aa1n02x5               g136(.a(new_n214), .o1(new_n232));
  oai012aa1n02x5               g137(.a(new_n221), .b(new_n220), .c(new_n218), .o1(new_n233));
  aobi12aa1n03x7               g138(.a(new_n233), .b(new_n230), .c(new_n232), .out0(new_n234));
  aoai13aa1n06x5               g139(.a(new_n234), .b(new_n231), .c(new_n195), .d(new_n200), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n04x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  nanp02aa1n02x5               g142(.a(\b[20] ), .b(\a[21] ), .o1(new_n238));
  xorc02aa1n02x5               g143(.a(\a[22] ), .b(\b[21] ), .out0(new_n239));
  aoi112aa1n03x4               g144(.a(new_n237), .b(new_n239), .c(new_n235), .d(new_n238), .o1(new_n240));
  inv000aa1n02x5               g145(.a(new_n237), .o1(new_n241));
  norb02aa1n02x5               g146(.a(new_n238), .b(new_n237), .out0(new_n242));
  nanp02aa1n02x5               g147(.a(new_n235), .b(new_n242), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n239), .o1(new_n244));
  tech160nm_fiaoi012aa1n05x5   g149(.a(new_n244), .b(new_n243), .c(new_n241), .o1(new_n245));
  norp02aa1n03x5               g150(.a(new_n245), .b(new_n240), .o1(\s[22] ));
  norp02aa1n02x5               g151(.a(\b[21] ), .b(\a[22] ), .o1(new_n247));
  nanp02aa1n02x5               g152(.a(\b[21] ), .b(\a[22] ), .o1(new_n248));
  nano23aa1n03x7               g153(.a(new_n237), .b(new_n247), .c(new_n248), .d(new_n238), .out0(new_n249));
  nand23aa1n03x5               g154(.a(new_n212), .b(new_n230), .c(new_n249), .o1(new_n250));
  nona23aa1n02x4               g155(.a(new_n221), .b(new_n219), .c(new_n218), .d(new_n220), .out0(new_n251));
  oai012aa1n06x5               g156(.a(new_n233), .b(new_n251), .c(new_n214), .o1(new_n252));
  oaoi03aa1n02x5               g157(.a(\a[22] ), .b(\b[21] ), .c(new_n241), .o1(new_n253));
  aoi012aa1n02x5               g158(.a(new_n253), .b(new_n252), .c(new_n249), .o1(new_n254));
  aoai13aa1n06x5               g159(.a(new_n254), .b(new_n250), .c(new_n195), .d(new_n200), .o1(new_n255));
  xorb03aa1n02x5               g160(.a(new_n255), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor022aa1n12x5               g161(.a(\b[22] ), .b(\a[23] ), .o1(new_n257));
  nanp02aa1n02x5               g162(.a(\b[22] ), .b(\a[23] ), .o1(new_n258));
  nor042aa1n02x5               g163(.a(\b[23] ), .b(\a[24] ), .o1(new_n259));
  nanp02aa1n02x5               g164(.a(\b[23] ), .b(\a[24] ), .o1(new_n260));
  norb02aa1n02x5               g165(.a(new_n260), .b(new_n259), .out0(new_n261));
  aoi112aa1n03x4               g166(.a(new_n257), .b(new_n261), .c(new_n255), .d(new_n258), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n257), .o1(new_n263));
  norb02aa1n02x5               g168(.a(new_n258), .b(new_n257), .out0(new_n264));
  nanp02aa1n02x5               g169(.a(new_n255), .b(new_n264), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n261), .o1(new_n266));
  tech160nm_fiaoi012aa1n02p5x5 g171(.a(new_n266), .b(new_n265), .c(new_n263), .o1(new_n267));
  nor002aa1n02x5               g172(.a(new_n267), .b(new_n262), .o1(\s[24] ));
  nano23aa1n06x5               g173(.a(new_n257), .b(new_n259), .c(new_n260), .d(new_n258), .out0(new_n269));
  nanb03aa1n02x5               g174(.a(new_n231), .b(new_n269), .c(new_n249), .out0(new_n270));
  nona22aa1n02x4               g175(.a(new_n260), .b(new_n259), .c(new_n257), .out0(new_n271));
  aoi022aa1n06x5               g176(.a(new_n269), .b(new_n253), .c(new_n271), .d(new_n260), .o1(new_n272));
  inv020aa1n03x5               g177(.a(new_n272), .o1(new_n273));
  aoi013aa1n02x4               g178(.a(new_n273), .b(new_n252), .c(new_n249), .d(new_n269), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n274), .b(new_n270), .c(new_n195), .d(new_n200), .o1(new_n275));
  xorb03aa1n02x5               g180(.a(new_n275), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n06x5               g181(.a(\b[24] ), .b(\a[25] ), .o1(new_n277));
  nanp02aa1n02x5               g182(.a(\b[24] ), .b(\a[25] ), .o1(new_n278));
  xnrc02aa1n12x5               g183(.a(\b[25] ), .b(\a[26] ), .out0(new_n279));
  inv000aa1d42x5               g184(.a(new_n279), .o1(new_n280));
  aoi112aa1n02x7               g185(.a(new_n277), .b(new_n280), .c(new_n275), .d(new_n278), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n277), .o1(new_n282));
  norb02aa1n02x5               g187(.a(new_n278), .b(new_n277), .out0(new_n283));
  nanp02aa1n03x5               g188(.a(new_n275), .b(new_n283), .o1(new_n284));
  aoi012aa1n03x5               g189(.a(new_n279), .b(new_n284), .c(new_n282), .o1(new_n285));
  norp02aa1n03x5               g190(.a(new_n285), .b(new_n281), .o1(\s[26] ));
  inv000aa1d42x5               g191(.a(new_n100), .o1(new_n287));
  oaih12aa1n02x5               g192(.a(new_n104), .b(new_n111), .c(new_n107), .o1(new_n288));
  nanp02aa1n02x5               g193(.a(new_n288), .b(new_n287), .o1(new_n289));
  nano23aa1n02x4               g194(.a(new_n125), .b(new_n120), .c(new_n119), .d(new_n122), .out0(new_n290));
  nanb03aa1n02x5               g195(.a(new_n118), .b(new_n290), .c(new_n113), .out0(new_n291));
  inv000aa1n02x5               g196(.a(new_n130), .o1(new_n292));
  tech160nm_fioai012aa1n04x5   g197(.a(new_n292), .b(new_n289), .c(new_n291), .o1(new_n293));
  nanp02aa1n02x5               g198(.a(new_n164), .b(new_n197), .o1(new_n294));
  nona22aa1n02x4               g199(.a(new_n294), .b(new_n199), .c(new_n198), .out0(new_n295));
  nano22aa1n12x5               g200(.a(new_n279), .b(new_n282), .c(new_n278), .out0(new_n296));
  nano22aa1n03x7               g201(.a(new_n250), .b(new_n269), .c(new_n296), .out0(new_n297));
  aoai13aa1n06x5               g202(.a(new_n297), .b(new_n295), .c(new_n293), .d(new_n194), .o1(new_n298));
  nano22aa1n03x5               g203(.a(new_n234), .b(new_n249), .c(new_n269), .out0(new_n299));
  oaoi03aa1n12x5               g204(.a(\a[26] ), .b(\b[25] ), .c(new_n282), .o1(new_n300));
  oaoi13aa1n06x5               g205(.a(new_n300), .b(new_n296), .c(new_n299), .d(new_n273), .o1(new_n301));
  nor042aa1n03x5               g206(.a(\b[26] ), .b(\a[27] ), .o1(new_n302));
  nanp02aa1n02x5               g207(.a(\b[26] ), .b(\a[27] ), .o1(new_n303));
  norb02aa1n15x5               g208(.a(new_n303), .b(new_n302), .out0(new_n304));
  xnbna2aa1n03x5               g209(.a(new_n304), .b(new_n298), .c(new_n301), .out0(\s[27] ));
  inv000aa1n06x5               g210(.a(new_n302), .o1(new_n306));
  inv000aa1d42x5               g211(.a(new_n304), .o1(new_n307));
  tech160nm_fiaoi012aa1n02p5x5 g212(.a(new_n307), .b(new_n298), .c(new_n301), .o1(new_n308));
  xnrc02aa1n12x5               g213(.a(\b[27] ), .b(\a[28] ), .out0(new_n309));
  nano22aa1n02x4               g214(.a(new_n308), .b(new_n306), .c(new_n309), .out0(new_n310));
  nanp03aa1n02x5               g215(.a(new_n252), .b(new_n249), .c(new_n269), .o1(new_n311));
  inv000aa1d42x5               g216(.a(new_n296), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n300), .o1(new_n313));
  aoai13aa1n04x5               g218(.a(new_n313), .b(new_n312), .c(new_n311), .d(new_n272), .o1(new_n314));
  aoai13aa1n03x5               g219(.a(new_n304), .b(new_n314), .c(new_n201), .d(new_n297), .o1(new_n315));
  aoi012aa1n03x5               g220(.a(new_n309), .b(new_n315), .c(new_n306), .o1(new_n316));
  norp02aa1n03x5               g221(.a(new_n316), .b(new_n310), .o1(\s[28] ));
  xnrc02aa1n12x5               g222(.a(\b[28] ), .b(\a[29] ), .out0(new_n318));
  nano22aa1n12x5               g223(.a(new_n309), .b(new_n306), .c(new_n303), .out0(new_n319));
  aoai13aa1n03x5               g224(.a(new_n319), .b(new_n314), .c(new_n201), .d(new_n297), .o1(new_n320));
  oao003aa1n02x5               g225(.a(\a[28] ), .b(\b[27] ), .c(new_n306), .carry(new_n321));
  aoi012aa1n03x5               g226(.a(new_n318), .b(new_n320), .c(new_n321), .o1(new_n322));
  inv000aa1d42x5               g227(.a(new_n319), .o1(new_n323));
  aoi012aa1n02x7               g228(.a(new_n323), .b(new_n298), .c(new_n301), .o1(new_n324));
  nano22aa1n02x4               g229(.a(new_n324), .b(new_n318), .c(new_n321), .out0(new_n325));
  norp02aa1n03x5               g230(.a(new_n322), .b(new_n325), .o1(\s[29] ));
  xorb03aa1n02x5               g231(.a(new_n110), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1d15x5               g232(.a(new_n318), .b(new_n309), .c(new_n303), .d(new_n306), .out0(new_n328));
  aoai13aa1n03x5               g233(.a(new_n328), .b(new_n314), .c(new_n201), .d(new_n297), .o1(new_n329));
  oao003aa1n02x5               g234(.a(\a[29] ), .b(\b[28] ), .c(new_n321), .carry(new_n330));
  xnrc02aa1n02x5               g235(.a(\b[29] ), .b(\a[30] ), .out0(new_n331));
  aoi012aa1n03x5               g236(.a(new_n331), .b(new_n329), .c(new_n330), .o1(new_n332));
  inv000aa1d42x5               g237(.a(new_n328), .o1(new_n333));
  tech160nm_fiaoi012aa1n02p5x5 g238(.a(new_n333), .b(new_n298), .c(new_n301), .o1(new_n334));
  nano22aa1n02x4               g239(.a(new_n334), .b(new_n330), .c(new_n331), .out0(new_n335));
  norp02aa1n03x5               g240(.a(new_n332), .b(new_n335), .o1(\s[30] ));
  norb03aa1d15x5               g241(.a(new_n319), .b(new_n318), .c(new_n331), .out0(new_n337));
  inv000aa1d42x5               g242(.a(new_n337), .o1(new_n338));
  tech160nm_fiaoi012aa1n02p5x5 g243(.a(new_n338), .b(new_n298), .c(new_n301), .o1(new_n339));
  oao003aa1n02x5               g244(.a(\a[30] ), .b(\b[29] ), .c(new_n330), .carry(new_n340));
  xnrc02aa1n02x5               g245(.a(\b[30] ), .b(\a[31] ), .out0(new_n341));
  nano22aa1n03x5               g246(.a(new_n339), .b(new_n340), .c(new_n341), .out0(new_n342));
  aoai13aa1n03x5               g247(.a(new_n337), .b(new_n314), .c(new_n201), .d(new_n297), .o1(new_n343));
  aoi012aa1n03x5               g248(.a(new_n341), .b(new_n343), .c(new_n340), .o1(new_n344));
  nor002aa1n02x5               g249(.a(new_n344), .b(new_n342), .o1(\s[31] ));
  xnbna2aa1n03x5               g250(.a(new_n111), .b(new_n105), .c(new_n106), .out0(\s[3] ));
  xnrc02aa1n02x5               g251(.a(\b[3] ), .b(\a[4] ), .out0(new_n347));
  oaoi03aa1n02x5               g252(.a(\a[3] ), .b(\b[2] ), .c(new_n111), .o1(new_n348));
  aob012aa1n02x5               g253(.a(new_n288), .b(new_n348), .c(new_n347), .out0(\s[4] ));
  xorb03aa1n02x5               g254(.a(new_n112), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi013aa1n03x5               g255(.a(new_n128), .b(new_n288), .c(new_n113), .d(new_n287), .o1(new_n351));
  xnbna2aa1n03x5               g256(.a(new_n351), .b(new_n116), .c(new_n117), .out0(\s[6] ));
  tech160nm_fioaoi03aa1n03p5x5 g257(.a(\a[6] ), .b(\b[5] ), .c(new_n351), .o1(new_n353));
  xorb03aa1n02x5               g258(.a(new_n353), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n03x5               g259(.a(new_n125), .b(new_n353), .c(new_n119), .o1(new_n355));
  xnbna2aa1n03x5               g260(.a(new_n355), .b(new_n121), .c(new_n122), .out0(\s[8] ));
  xorb03aa1n02x5               g261(.a(new_n293), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


