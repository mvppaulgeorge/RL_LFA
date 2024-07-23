// Benchmark "adder" written by ABC on Wed Jul 17 20:13:08 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n166, new_n167, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n202,
    new_n203, new_n204, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n327, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n347, new_n350, new_n352, new_n353, new_n355;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d28x5               g001(.a(\a[9] ), .o1(new_n97));
  inv040aa1d24x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  nor002aa1d32x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nand02aa1n08x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nanp02aa1n12x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nor002aa1n20x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nona23aa1n09x5               g008(.a(new_n102), .b(new_n101), .c(new_n103), .d(new_n100), .out0(new_n104));
  xnrc02aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .out0(new_n105));
  tech160nm_fixnrc02aa1n04x5   g010(.a(\b[4] ), .b(\a[5] ), .out0(new_n106));
  nor043aa1n06x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n107));
  and002aa1n06x5               g012(.a(\b[3] ), .b(\a[4] ), .o(new_n108));
  inv040aa1d32x5               g013(.a(\a[3] ), .o1(new_n109));
  inv040aa1n16x5               g014(.a(\b[2] ), .o1(new_n110));
  nand02aa1n06x5               g015(.a(new_n110), .b(new_n109), .o1(new_n111));
  nanp02aa1n06x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nand02aa1d04x5               g017(.a(new_n111), .b(new_n112), .o1(new_n113));
  nor042aa1n12x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  nand42aa1d28x5               g019(.a(\b[0] ), .b(\a[1] ), .o1(new_n115));
  nand42aa1d28x5               g020(.a(\b[1] ), .b(\a[2] ), .o1(new_n116));
  aoi012aa1d24x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  oa0022aa1n03x5               g022(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n118));
  oaoi13aa1n12x5               g023(.a(new_n108), .b(new_n118), .c(new_n117), .d(new_n113), .o1(new_n119));
  aoi112aa1n02x5               g024(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n120));
  norb02aa1n12x5               g025(.a(new_n101), .b(new_n100), .out0(new_n121));
  norb02aa1n06x5               g026(.a(new_n102), .b(new_n103), .out0(new_n122));
  norp02aa1n02x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  aoi112aa1n09x5               g028(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n124));
  oai112aa1n06x5               g029(.a(new_n122), .b(new_n121), .c(new_n124), .d(new_n123), .o1(new_n125));
  nona22aa1n06x5               g030(.a(new_n125), .b(new_n120), .c(new_n100), .out0(new_n126));
  xorc02aa1n02x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoai13aa1n06x5               g032(.a(new_n127), .b(new_n126), .c(new_n119), .d(new_n107), .o1(new_n128));
  nor002aa1d32x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand02aa1d28x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n128), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g037(.a(new_n129), .o1(new_n133));
  inv000aa1n02x5               g038(.a(new_n130), .o1(new_n134));
  nor002aa1d32x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand42aa1d28x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  inv000aa1n02x5               g042(.a(new_n137), .o1(new_n138));
  aoi113aa1n02x5               g043(.a(new_n138), .b(new_n134), .c(new_n128), .d(new_n133), .e(new_n99), .o1(new_n139));
  nanp02aa1n03x5               g044(.a(new_n128), .b(new_n99), .o1(new_n140));
  aoai13aa1n12x5               g045(.a(new_n130), .b(new_n129), .c(new_n97), .d(new_n98), .o1(new_n141));
  inv000aa1d42x5               g046(.a(new_n141), .o1(new_n142));
  aoi112aa1n02x5               g047(.a(new_n142), .b(new_n137), .c(new_n140), .d(new_n131), .o1(new_n143));
  norp02aa1n02x5               g048(.a(new_n143), .b(new_n139), .o1(\s[11] ));
  inv000aa1d42x5               g049(.a(new_n135), .o1(new_n145));
  nor042aa1n04x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nand42aa1n20x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  nano22aa1n02x4               g054(.a(new_n139), .b(new_n145), .c(new_n149), .out0(new_n150));
  aoai13aa1n03x5               g055(.a(new_n137), .b(new_n142), .c(new_n140), .d(new_n131), .o1(new_n151));
  tech160nm_fiaoi012aa1n02p5x5 g056(.a(new_n149), .b(new_n151), .c(new_n145), .o1(new_n152));
  norp02aa1n03x5               g057(.a(new_n152), .b(new_n150), .o1(\s[12] ));
  and002aa1n18x5               g058(.a(\b[8] ), .b(\a[9] ), .o(new_n154));
  inv040aa1d30x5               g059(.a(new_n154), .o1(new_n155));
  nona23aa1n09x5               g060(.a(new_n147), .b(new_n136), .c(new_n135), .d(new_n146), .out0(new_n156));
  nano32aa1n03x7               g061(.a(new_n156), .b(new_n131), .c(new_n155), .d(new_n99), .out0(new_n157));
  aoai13aa1n06x5               g062(.a(new_n157), .b(new_n126), .c(new_n119), .d(new_n107), .o1(new_n158));
  tech160nm_fioai012aa1n03p5x5 g063(.a(new_n147), .b(new_n146), .c(new_n135), .o1(new_n159));
  oai012aa1n12x5               g064(.a(new_n159), .b(new_n156), .c(new_n141), .o1(new_n160));
  inv030aa1n02x5               g065(.a(new_n160), .o1(new_n161));
  nor002aa1d32x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nanp02aa1n09x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  nanb02aa1n02x5               g068(.a(new_n162), .b(new_n163), .out0(new_n164));
  xobna2aa1n03x5               g069(.a(new_n164), .b(new_n158), .c(new_n161), .out0(\s[13] ));
  inv000aa1d42x5               g070(.a(new_n162), .o1(new_n166));
  aoai13aa1n03x5               g071(.a(new_n166), .b(new_n164), .c(new_n158), .d(new_n161), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n08x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nanp02aa1n04x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  oai012aa1n02x5               g075(.a(new_n170), .b(new_n169), .c(new_n162), .o1(new_n171));
  nona23aa1n03x5               g076(.a(new_n170), .b(new_n163), .c(new_n162), .d(new_n169), .out0(new_n172));
  aoai13aa1n04x5               g077(.a(new_n171), .b(new_n172), .c(new_n158), .d(new_n161), .o1(new_n173));
  xorb03aa1n02x5               g078(.a(new_n173), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nand42aa1n08x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nanb02aa1n02x5               g081(.a(new_n175), .b(new_n176), .out0(new_n177));
  inv000aa1d42x5               g082(.a(new_n177), .o1(new_n178));
  nor002aa1d32x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  inv000aa1d42x5               g084(.a(new_n179), .o1(new_n180));
  nanp02aa1n09x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  aoi122aa1n02x7               g086(.a(new_n175), .b(new_n181), .c(new_n180), .d(new_n173), .e(new_n178), .o1(new_n182));
  inv000aa1d42x5               g087(.a(new_n175), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(new_n173), .b(new_n178), .o1(new_n184));
  nanb02aa1n02x5               g089(.a(new_n179), .b(new_n181), .out0(new_n185));
  aoi012aa1n03x5               g090(.a(new_n185), .b(new_n184), .c(new_n183), .o1(new_n186));
  norp02aa1n02x5               g091(.a(new_n186), .b(new_n182), .o1(\s[16] ));
  aoi112aa1n03x5               g092(.a(new_n134), .b(new_n129), .c(new_n97), .d(new_n98), .o1(new_n188));
  nano23aa1n03x7               g093(.a(new_n135), .b(new_n146), .c(new_n147), .d(new_n136), .out0(new_n189));
  nano23aa1n03x7               g094(.a(new_n162), .b(new_n169), .c(new_n170), .d(new_n163), .out0(new_n190));
  nano23aa1n03x7               g095(.a(new_n175), .b(new_n179), .c(new_n181), .d(new_n176), .out0(new_n191));
  nand02aa1d04x5               g096(.a(new_n191), .b(new_n190), .o1(new_n192));
  nano32aa1n03x7               g097(.a(new_n192), .b(new_n189), .c(new_n188), .d(new_n155), .out0(new_n193));
  aoai13aa1n12x5               g098(.a(new_n193), .b(new_n126), .c(new_n119), .d(new_n107), .o1(new_n194));
  nona23aa1n09x5               g099(.a(new_n181), .b(new_n176), .c(new_n175), .d(new_n179), .out0(new_n195));
  nor042aa1n02x5               g100(.a(new_n195), .b(new_n172), .o1(new_n196));
  aoi112aa1n03x5               g101(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n197));
  tech160nm_fioai012aa1n04x5   g102(.a(new_n180), .b(new_n195), .c(new_n171), .o1(new_n198));
  aoi112aa1n09x5               g103(.a(new_n198), .b(new_n197), .c(new_n160), .d(new_n196), .o1(new_n199));
  nand02aa1d08x5               g104(.a(new_n194), .b(new_n199), .o1(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g106(.a(\a[17] ), .o1(new_n202));
  inv000aa1d42x5               g107(.a(\b[16] ), .o1(new_n203));
  oaoi03aa1n03x5               g108(.a(new_n202), .b(new_n203), .c(new_n200), .o1(new_n204));
  xnrb03aa1n03x5               g109(.a(new_n204), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nanp02aa1n02x5               g110(.a(new_n203), .b(new_n202), .o1(new_n206));
  nanp02aa1n02x5               g111(.a(\b[16] ), .b(\a[17] ), .o1(new_n207));
  nor002aa1d32x5               g112(.a(\b[17] ), .b(\a[18] ), .o1(new_n208));
  nand02aa1n08x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  nanb02aa1n06x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  nano22aa1n12x5               g115(.a(new_n210), .b(new_n206), .c(new_n207), .out0(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  aoai13aa1n12x5               g117(.a(new_n209), .b(new_n208), .c(new_n202), .d(new_n203), .o1(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n212), .c(new_n194), .d(new_n199), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n16x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  nand22aa1n03x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  nor042aa1n04x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nanp02aa1n02x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  nanb02aa1n02x5               g125(.a(new_n219), .b(new_n220), .out0(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  aoi112aa1n02x7               g127(.a(new_n222), .b(new_n217), .c(new_n214), .d(new_n218), .o1(new_n223));
  inv000aa1d42x5               g128(.a(new_n217), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n218), .b(new_n217), .out0(new_n225));
  tech160nm_finand02aa1n05x5   g130(.a(new_n214), .b(new_n225), .o1(new_n226));
  aoi012aa1n03x5               g131(.a(new_n221), .b(new_n226), .c(new_n224), .o1(new_n227));
  nor002aa1n02x5               g132(.a(new_n227), .b(new_n223), .o1(\s[20] ));
  nano23aa1n06x5               g133(.a(new_n217), .b(new_n219), .c(new_n220), .d(new_n218), .out0(new_n229));
  nanp02aa1n02x5               g134(.a(new_n211), .b(new_n229), .o1(new_n230));
  inv000aa1n02x5               g135(.a(new_n213), .o1(new_n231));
  oaih12aa1n02x5               g136(.a(new_n220), .b(new_n219), .c(new_n217), .o1(new_n232));
  aobi12aa1n03x7               g137(.a(new_n232), .b(new_n229), .c(new_n231), .out0(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n230), .c(new_n194), .d(new_n199), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n04x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  xorc02aa1n02x5               g141(.a(\a[21] ), .b(\b[20] ), .out0(new_n237));
  xorc02aa1n02x5               g142(.a(\a[22] ), .b(\b[21] ), .out0(new_n238));
  aoi112aa1n03x4               g143(.a(new_n236), .b(new_n238), .c(new_n234), .d(new_n237), .o1(new_n239));
  inv000aa1n02x5               g144(.a(new_n236), .o1(new_n240));
  nanp02aa1n02x5               g145(.a(new_n234), .b(new_n237), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n238), .o1(new_n242));
  aoi012aa1n03x5               g147(.a(new_n242), .b(new_n241), .c(new_n240), .o1(new_n243));
  norp02aa1n03x5               g148(.a(new_n243), .b(new_n239), .o1(\s[22] ));
  inv000aa1d42x5               g149(.a(\a[21] ), .o1(new_n245));
  inv000aa1d42x5               g150(.a(\a[22] ), .o1(new_n246));
  xroi22aa1d04x5               g151(.a(new_n245), .b(\b[20] ), .c(new_n246), .d(\b[21] ), .out0(new_n247));
  nand23aa1n03x5               g152(.a(new_n247), .b(new_n211), .c(new_n229), .o1(new_n248));
  nona23aa1n03x5               g153(.a(new_n220), .b(new_n218), .c(new_n217), .d(new_n219), .out0(new_n249));
  oai012aa1n06x5               g154(.a(new_n232), .b(new_n249), .c(new_n213), .o1(new_n250));
  tech160nm_fioaoi03aa1n02p5x5 g155(.a(\a[22] ), .b(\b[21] ), .c(new_n240), .o1(new_n251));
  aoi012aa1n02x5               g156(.a(new_n251), .b(new_n250), .c(new_n247), .o1(new_n252));
  aoai13aa1n06x5               g157(.a(new_n252), .b(new_n248), .c(new_n194), .d(new_n199), .o1(new_n253));
  xorb03aa1n02x5               g158(.a(new_n253), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor022aa1n16x5               g159(.a(\b[22] ), .b(\a[23] ), .o1(new_n255));
  nand22aa1n04x5               g160(.a(\b[22] ), .b(\a[23] ), .o1(new_n256));
  nor042aa1n02x5               g161(.a(\b[23] ), .b(\a[24] ), .o1(new_n257));
  nand02aa1n04x5               g162(.a(\b[23] ), .b(\a[24] ), .o1(new_n258));
  norb02aa1n02x5               g163(.a(new_n258), .b(new_n257), .out0(new_n259));
  aoi112aa1n03x4               g164(.a(new_n255), .b(new_n259), .c(new_n253), .d(new_n256), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n255), .o1(new_n261));
  norb02aa1n02x5               g166(.a(new_n256), .b(new_n255), .out0(new_n262));
  nanp02aa1n03x5               g167(.a(new_n253), .b(new_n262), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n259), .o1(new_n264));
  aoi012aa1n03x5               g169(.a(new_n264), .b(new_n263), .c(new_n261), .o1(new_n265));
  nor002aa1n02x5               g170(.a(new_n265), .b(new_n260), .o1(\s[24] ));
  nano23aa1n06x5               g171(.a(new_n255), .b(new_n257), .c(new_n258), .d(new_n256), .out0(new_n267));
  nanb03aa1n02x5               g172(.a(new_n230), .b(new_n267), .c(new_n247), .out0(new_n268));
  nona22aa1n02x4               g173(.a(new_n258), .b(new_n257), .c(new_n255), .out0(new_n269));
  aoi022aa1n06x5               g174(.a(new_n267), .b(new_n251), .c(new_n269), .d(new_n258), .o1(new_n270));
  inv000aa1n02x5               g175(.a(new_n270), .o1(new_n271));
  aoi013aa1n02x4               g176(.a(new_n271), .b(new_n250), .c(new_n247), .d(new_n267), .o1(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n268), .c(new_n194), .d(new_n199), .o1(new_n273));
  xorb03aa1n02x5               g178(.a(new_n273), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n04x5               g179(.a(\b[24] ), .b(\a[25] ), .o1(new_n275));
  xorc02aa1n06x5               g180(.a(\a[25] ), .b(\b[24] ), .out0(new_n276));
  xorc02aa1n12x5               g181(.a(\a[26] ), .b(\b[25] ), .out0(new_n277));
  aoi112aa1n02x7               g182(.a(new_n275), .b(new_n277), .c(new_n273), .d(new_n276), .o1(new_n278));
  inv000aa1n03x5               g183(.a(new_n275), .o1(new_n279));
  nand02aa1n03x5               g184(.a(new_n273), .b(new_n276), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n277), .o1(new_n281));
  aoi012aa1n03x5               g186(.a(new_n281), .b(new_n280), .c(new_n279), .o1(new_n282));
  norp02aa1n03x5               g187(.a(new_n282), .b(new_n278), .o1(\s[26] ));
  nano23aa1n02x4               g188(.a(new_n103), .b(new_n100), .c(new_n101), .d(new_n102), .out0(new_n284));
  nona22aa1n02x4               g189(.a(new_n284), .b(new_n105), .c(new_n106), .out0(new_n285));
  inv000aa1n02x5               g190(.a(new_n108), .o1(new_n286));
  xorc02aa1n02x5               g191(.a(\a[3] ), .b(\b[2] ), .out0(new_n287));
  nanp02aa1n02x5               g192(.a(new_n116), .b(new_n115), .o1(new_n288));
  oai012aa1n02x5               g193(.a(new_n288), .b(\b[1] ), .c(\a[2] ), .o1(new_n289));
  inv040aa1n02x5               g194(.a(new_n118), .o1(new_n290));
  aoai13aa1n02x5               g195(.a(new_n286), .b(new_n290), .c(new_n289), .d(new_n287), .o1(new_n291));
  inv000aa1d42x5               g196(.a(\a[5] ), .o1(new_n292));
  inv000aa1d42x5               g197(.a(\b[4] ), .o1(new_n293));
  nanp02aa1n02x5               g198(.a(new_n293), .b(new_n292), .o1(new_n294));
  oaoi03aa1n02x5               g199(.a(\a[6] ), .b(\b[5] ), .c(new_n294), .o1(new_n295));
  aoi112aa1n02x5               g200(.a(new_n120), .b(new_n100), .c(new_n284), .d(new_n295), .o1(new_n296));
  oai012aa1n02x7               g201(.a(new_n296), .b(new_n291), .c(new_n285), .o1(new_n297));
  nanp02aa1n03x5               g202(.a(new_n160), .b(new_n196), .o1(new_n298));
  nona22aa1n03x5               g203(.a(new_n298), .b(new_n198), .c(new_n197), .out0(new_n299));
  and002aa1n09x5               g204(.a(new_n277), .b(new_n276), .o(new_n300));
  nano22aa1n03x7               g205(.a(new_n248), .b(new_n300), .c(new_n267), .out0(new_n301));
  aoai13aa1n06x5               g206(.a(new_n301), .b(new_n299), .c(new_n297), .d(new_n193), .o1(new_n302));
  nano22aa1n03x5               g207(.a(new_n233), .b(new_n247), .c(new_n267), .out0(new_n303));
  oaoi03aa1n09x5               g208(.a(\a[26] ), .b(\b[25] ), .c(new_n279), .o1(new_n304));
  oaoi13aa1n09x5               g209(.a(new_n304), .b(new_n300), .c(new_n303), .d(new_n271), .o1(new_n305));
  nor042aa1n03x5               g210(.a(\b[26] ), .b(\a[27] ), .o1(new_n306));
  nanp02aa1n02x5               g211(.a(\b[26] ), .b(\a[27] ), .o1(new_n307));
  norb02aa1n02x5               g212(.a(new_n307), .b(new_n306), .out0(new_n308));
  xnbna2aa1n03x5               g213(.a(new_n308), .b(new_n302), .c(new_n305), .out0(\s[27] ));
  inv040aa1n03x5               g214(.a(new_n306), .o1(new_n310));
  aobi12aa1n02x5               g215(.a(new_n308), .b(new_n302), .c(new_n305), .out0(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[27] ), .b(\a[28] ), .out0(new_n312));
  nano22aa1n02x4               g217(.a(new_n311), .b(new_n310), .c(new_n312), .out0(new_n313));
  nanp03aa1n02x5               g218(.a(new_n250), .b(new_n247), .c(new_n267), .o1(new_n314));
  inv000aa1n02x5               g219(.a(new_n300), .o1(new_n315));
  inv000aa1d42x5               g220(.a(new_n304), .o1(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n315), .c(new_n314), .d(new_n270), .o1(new_n317));
  aoai13aa1n03x5               g222(.a(new_n308), .b(new_n317), .c(new_n200), .d(new_n301), .o1(new_n318));
  aoi012aa1n03x5               g223(.a(new_n312), .b(new_n318), .c(new_n310), .o1(new_n319));
  nor002aa1n02x5               g224(.a(new_n319), .b(new_n313), .o1(\s[28] ));
  xnrc02aa1n02x5               g225(.a(\b[28] ), .b(\a[29] ), .out0(new_n321));
  nano22aa1n02x4               g226(.a(new_n312), .b(new_n310), .c(new_n307), .out0(new_n322));
  aoai13aa1n03x5               g227(.a(new_n322), .b(new_n317), .c(new_n200), .d(new_n301), .o1(new_n323));
  oao003aa1n02x5               g228(.a(\a[28] ), .b(\b[27] ), .c(new_n310), .carry(new_n324));
  aoi012aa1n03x5               g229(.a(new_n321), .b(new_n323), .c(new_n324), .o1(new_n325));
  aobi12aa1n02x5               g230(.a(new_n322), .b(new_n302), .c(new_n305), .out0(new_n326));
  nano22aa1n02x4               g231(.a(new_n326), .b(new_n321), .c(new_n324), .out0(new_n327));
  nor002aa1n02x5               g232(.a(new_n325), .b(new_n327), .o1(\s[29] ));
  xorb03aa1n02x5               g233(.a(new_n115), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g234(.a(new_n321), .b(new_n312), .c(new_n307), .d(new_n310), .out0(new_n330));
  aoai13aa1n03x5               g235(.a(new_n330), .b(new_n317), .c(new_n200), .d(new_n301), .o1(new_n331));
  oao003aa1n02x5               g236(.a(\a[29] ), .b(\b[28] ), .c(new_n324), .carry(new_n332));
  xnrc02aa1n02x5               g237(.a(\b[29] ), .b(\a[30] ), .out0(new_n333));
  aoi012aa1n03x5               g238(.a(new_n333), .b(new_n331), .c(new_n332), .o1(new_n334));
  aobi12aa1n02x5               g239(.a(new_n330), .b(new_n302), .c(new_n305), .out0(new_n335));
  nano22aa1n02x4               g240(.a(new_n335), .b(new_n332), .c(new_n333), .out0(new_n336));
  nor002aa1n02x5               g241(.a(new_n334), .b(new_n336), .o1(\s[30] ));
  xnrc02aa1n02x5               g242(.a(\b[30] ), .b(\a[31] ), .out0(new_n338));
  norb03aa1n02x5               g243(.a(new_n322), .b(new_n321), .c(new_n333), .out0(new_n339));
  aobi12aa1n02x5               g244(.a(new_n339), .b(new_n302), .c(new_n305), .out0(new_n340));
  oao003aa1n02x5               g245(.a(\a[30] ), .b(\b[29] ), .c(new_n332), .carry(new_n341));
  nano22aa1n02x4               g246(.a(new_n340), .b(new_n338), .c(new_n341), .out0(new_n342));
  aoai13aa1n03x5               g247(.a(new_n339), .b(new_n317), .c(new_n200), .d(new_n301), .o1(new_n343));
  aoi012aa1n03x5               g248(.a(new_n338), .b(new_n343), .c(new_n341), .o1(new_n344));
  nor002aa1n02x5               g249(.a(new_n344), .b(new_n342), .o1(\s[31] ));
  xnbna2aa1n03x5               g250(.a(new_n117), .b(new_n111), .c(new_n112), .out0(\s[3] ));
  oaoi03aa1n02x5               g251(.a(\a[3] ), .b(\b[2] ), .c(new_n117), .o1(new_n347));
  xorb03aa1n02x5               g252(.a(new_n347), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g253(.a(new_n119), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g254(.a(new_n292), .b(new_n293), .c(new_n119), .o1(new_n350));
  xnrb03aa1n02x5               g255(.a(new_n350), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoi112aa1n02x5               g256(.a(new_n291), .b(new_n106), .c(\a[6] ), .d(\b[5] ), .o1(new_n352));
  norp02aa1n02x5               g257(.a(new_n352), .b(new_n295), .o1(new_n353));
  xnrc02aa1n02x5               g258(.a(new_n353), .b(new_n122), .out0(\s[7] ));
  oaoi13aa1n02x5               g259(.a(new_n103), .b(new_n102), .c(new_n352), .d(new_n295), .o1(new_n355));
  xnrc02aa1n02x5               g260(.a(new_n355), .b(new_n121), .out0(\s[8] ));
  xorb03aa1n02x5               g261(.a(new_n297), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


