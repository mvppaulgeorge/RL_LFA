// Benchmark "adder" written by ABC on Thu Jul 18 05:55:28 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n166, new_n167, new_n168, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n280, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n331,
    new_n334, new_n335, new_n337, new_n338, new_n339, new_n341;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nand42aa1n03x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n09x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nor042aa1n04x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oaih12aa1n06x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  xnrc02aa1n12x5               g007(.a(\b[3] ), .b(\a[4] ), .out0(new_n103));
  inv000aa1d42x5               g008(.a(new_n103), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanb02aa1n06x5               g011(.a(new_n105), .b(new_n106), .out0(new_n107));
  nona22aa1n09x5               g012(.a(new_n104), .b(new_n102), .c(new_n107), .out0(new_n108));
  oai022aa1n02x7               g013(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n109));
  aob012aa1n02x5               g014(.a(new_n109), .b(\b[3] ), .c(\a[4] ), .out0(new_n110));
  nand42aa1n03x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nor022aa1n12x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand42aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nor022aa1n16x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nano23aa1n03x7               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .out0(new_n117));
  nona22aa1n02x4               g022(.a(new_n115), .b(new_n116), .c(new_n117), .out0(new_n118));
  oai012aa1n02x5               g023(.a(new_n113), .b(new_n114), .c(new_n112), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\a[6] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\b[5] ), .o1(new_n121));
  nor042aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  oao003aa1n02x5               g027(.a(new_n120), .b(new_n121), .c(new_n122), .carry(new_n123));
  aobi12aa1n06x5               g028(.a(new_n119), .b(new_n115), .c(new_n123), .out0(new_n124));
  aoai13aa1n04x5               g029(.a(new_n124), .b(new_n118), .c(new_n108), .d(new_n110), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(new_n97), .b(new_n98), .c(new_n125), .o1(new_n126));
  xnrb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand42aa1d28x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nor002aa1d32x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nand22aa1n12x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  nor042aa1n06x5               g036(.a(\b[8] ), .b(\a[9] ), .o1(new_n132));
  and002aa1n12x5               g037(.a(\b[8] ), .b(\a[9] ), .o(new_n133));
  nona22aa1n02x4               g038(.a(new_n125), .b(new_n133), .c(new_n132), .out0(new_n134));
  nor002aa1d32x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  nona22aa1n02x4               g040(.a(new_n134), .b(new_n135), .c(new_n132), .out0(new_n136));
  xobna2aa1n03x5               g041(.a(new_n131), .b(new_n136), .c(new_n128), .out0(\s[11] ));
  inv000aa1d42x5               g042(.a(new_n128), .o1(new_n138));
  inv020aa1n03x5               g043(.a(new_n130), .o1(new_n139));
  nona32aa1n02x4               g044(.a(new_n136), .b(new_n139), .c(new_n129), .d(new_n138), .out0(new_n140));
  nor002aa1d32x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand42aa1d28x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  nona22aa1n02x4               g048(.a(new_n140), .b(new_n143), .c(new_n129), .out0(new_n144));
  inv000aa1d42x5               g049(.a(new_n129), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n143), .o1(new_n146));
  aoi012aa1n02x5               g051(.a(new_n146), .b(new_n140), .c(new_n145), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n144), .b(new_n147), .out0(\s[12] ));
  oai013aa1n09x5               g053(.a(new_n110), .b(new_n103), .c(new_n102), .d(new_n107), .o1(new_n149));
  nona23aa1n03x5               g054(.a(new_n113), .b(new_n111), .c(new_n114), .d(new_n112), .out0(new_n150));
  nor043aa1n03x5               g055(.a(new_n150), .b(new_n116), .c(new_n117), .o1(new_n151));
  oaoi03aa1n02x5               g056(.a(new_n120), .b(new_n121), .c(new_n122), .o1(new_n152));
  tech160nm_fioai012aa1n05x5   g057(.a(new_n119), .b(new_n150), .c(new_n152), .o1(new_n153));
  norb03aa1d15x5               g058(.a(new_n128), .b(new_n132), .c(new_n135), .out0(new_n154));
  norb03aa1d15x5               g059(.a(new_n142), .b(new_n129), .c(new_n141), .out0(new_n155));
  nona23aa1d16x5               g060(.a(new_n154), .b(new_n155), .c(new_n139), .d(new_n133), .out0(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  aoai13aa1n03x5               g062(.a(new_n157), .b(new_n153), .c(new_n149), .d(new_n151), .o1(new_n158));
  nona23aa1d18x5               g063(.a(new_n142), .b(new_n130), .c(new_n129), .d(new_n141), .out0(new_n159));
  oaih12aa1n12x5               g064(.a(new_n142), .b(new_n141), .c(new_n129), .o1(new_n160));
  aoai13aa1n09x5               g065(.a(new_n128), .b(new_n135), .c(new_n97), .d(new_n98), .o1(new_n161));
  oai012aa1d24x5               g066(.a(new_n160), .b(new_n159), .c(new_n161), .o1(new_n162));
  inv000aa1d42x5               g067(.a(new_n162), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(new_n158), .b(new_n163), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g070(.a(\a[13] ), .o1(new_n166));
  inv000aa1d42x5               g071(.a(\b[12] ), .o1(new_n167));
  oaoi03aa1n02x5               g072(.a(new_n166), .b(new_n167), .c(new_n164), .o1(new_n168));
  xnrb03aa1n02x5               g073(.a(new_n168), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n08x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  tech160nm_finand02aa1n03p5x5 g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n171), .b(new_n170), .c(new_n166), .d(new_n167), .o1(new_n172));
  nor022aa1n04x5               g077(.a(\b[12] ), .b(\a[13] ), .o1(new_n173));
  tech160nm_finand02aa1n03p5x5 g078(.a(\b[12] ), .b(\a[13] ), .o1(new_n174));
  nona23aa1n03x5               g079(.a(new_n171), .b(new_n174), .c(new_n173), .d(new_n170), .out0(new_n175));
  aoai13aa1n03x5               g080(.a(new_n172), .b(new_n175), .c(new_n158), .d(new_n163), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor022aa1n08x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  nand42aa1n04x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  nor022aa1n06x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  nanp02aa1n04x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nanb02aa1n02x5               g086(.a(new_n180), .b(new_n181), .out0(new_n182));
  inv000aa1d42x5               g087(.a(new_n182), .o1(new_n183));
  aoi112aa1n02x5               g088(.a(new_n183), .b(new_n178), .c(new_n176), .d(new_n179), .o1(new_n184));
  aoai13aa1n03x5               g089(.a(new_n183), .b(new_n178), .c(new_n176), .d(new_n179), .o1(new_n185));
  norb02aa1n02x7               g090(.a(new_n185), .b(new_n184), .out0(\s[16] ));
  nano23aa1n03x7               g091(.a(new_n173), .b(new_n170), .c(new_n171), .d(new_n174), .out0(new_n187));
  nano23aa1n03x7               g092(.a(new_n178), .b(new_n180), .c(new_n181), .d(new_n179), .out0(new_n188));
  nano22aa1d15x5               g093(.a(new_n156), .b(new_n187), .c(new_n188), .out0(new_n189));
  aoai13aa1n12x5               g094(.a(new_n189), .b(new_n153), .c(new_n149), .d(new_n151), .o1(new_n190));
  nona23aa1n09x5               g095(.a(new_n181), .b(new_n179), .c(new_n178), .d(new_n180), .out0(new_n191));
  nor002aa1n03x5               g096(.a(new_n191), .b(new_n175), .o1(new_n192));
  oa0012aa1n02x5               g097(.a(new_n181), .b(new_n180), .c(new_n178), .o(new_n193));
  norp02aa1n02x5               g098(.a(new_n191), .b(new_n172), .o1(new_n194));
  aoi112aa1n09x5               g099(.a(new_n194), .b(new_n193), .c(new_n162), .d(new_n192), .o1(new_n195));
  nand02aa1d08x5               g100(.a(new_n190), .b(new_n195), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g102(.a(\a[17] ), .o1(new_n198));
  inv000aa1d42x5               g103(.a(\b[16] ), .o1(new_n199));
  oaoi03aa1n03x5               g104(.a(new_n198), .b(new_n199), .c(new_n196), .o1(new_n200));
  xnrb03aa1n03x5               g105(.a(new_n200), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nanp02aa1n02x5               g106(.a(new_n199), .b(new_n198), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  nor022aa1n06x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  nand42aa1n03x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nanb02aa1n06x5               g110(.a(new_n204), .b(new_n205), .out0(new_n206));
  nano22aa1n09x5               g111(.a(new_n206), .b(new_n202), .c(new_n203), .out0(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n205), .b(new_n204), .c(new_n198), .d(new_n199), .o1(new_n209));
  aoai13aa1n06x5               g114(.a(new_n209), .b(new_n208), .c(new_n190), .d(new_n195), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n12x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  tech160nm_finand02aa1n03p5x5 g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nor042aa1n03x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  nand42aa1n03x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  nanb02aa1n02x5               g121(.a(new_n215), .b(new_n216), .out0(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoi112aa1n03x4               g123(.a(new_n213), .b(new_n218), .c(new_n210), .d(new_n214), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n213), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n214), .b(new_n213), .out0(new_n221));
  nanp02aa1n03x5               g126(.a(new_n210), .b(new_n221), .o1(new_n222));
  tech160nm_fiaoi012aa1n02p5x5 g127(.a(new_n217), .b(new_n222), .c(new_n220), .o1(new_n223));
  norp02aa1n03x5               g128(.a(new_n223), .b(new_n219), .o1(\s[20] ));
  nano23aa1n09x5               g129(.a(new_n213), .b(new_n215), .c(new_n216), .d(new_n214), .out0(new_n225));
  nanp02aa1n02x5               g130(.a(new_n207), .b(new_n225), .o1(new_n226));
  inv000aa1n02x5               g131(.a(new_n209), .o1(new_n227));
  oai012aa1n02x5               g132(.a(new_n216), .b(new_n215), .c(new_n213), .o1(new_n228));
  aobi12aa1n06x5               g133(.a(new_n228), .b(new_n225), .c(new_n227), .out0(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n226), .c(new_n190), .d(new_n195), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  nand42aa1n03x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[22] ), .b(\b[21] ), .out0(new_n234));
  aoi112aa1n03x4               g139(.a(new_n232), .b(new_n234), .c(new_n230), .d(new_n233), .o1(new_n235));
  inv000aa1n02x5               g140(.a(new_n232), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n233), .b(new_n232), .out0(new_n237));
  nand02aa1d04x5               g142(.a(new_n230), .b(new_n237), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n234), .o1(new_n239));
  aoi012aa1n06x5               g144(.a(new_n239), .b(new_n238), .c(new_n236), .o1(new_n240));
  nor042aa1n03x5               g145(.a(new_n240), .b(new_n235), .o1(\s[22] ));
  norp02aa1n02x5               g146(.a(\b[21] ), .b(\a[22] ), .o1(new_n242));
  nanp02aa1n02x5               g147(.a(\b[21] ), .b(\a[22] ), .o1(new_n243));
  nano23aa1n09x5               g148(.a(new_n232), .b(new_n242), .c(new_n243), .d(new_n233), .out0(new_n244));
  nand23aa1n03x5               g149(.a(new_n207), .b(new_n225), .c(new_n244), .o1(new_n245));
  nona23aa1n03x5               g150(.a(new_n216), .b(new_n214), .c(new_n213), .d(new_n215), .out0(new_n246));
  oaih12aa1n02x5               g151(.a(new_n228), .b(new_n246), .c(new_n209), .o1(new_n247));
  oaoi03aa1n02x5               g152(.a(\a[22] ), .b(\b[21] ), .c(new_n236), .o1(new_n248));
  aoi012aa1n02x5               g153(.a(new_n248), .b(new_n247), .c(new_n244), .o1(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n245), .c(new_n190), .d(new_n195), .o1(new_n250));
  xorb03aa1n02x5               g155(.a(new_n250), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor022aa1n16x5               g156(.a(\b[22] ), .b(\a[23] ), .o1(new_n252));
  nanp02aa1n02x5               g157(.a(\b[22] ), .b(\a[23] ), .o1(new_n253));
  nor042aa1n02x5               g158(.a(\b[23] ), .b(\a[24] ), .o1(new_n254));
  nanp02aa1n04x5               g159(.a(\b[23] ), .b(\a[24] ), .o1(new_n255));
  norb02aa1n02x5               g160(.a(new_n255), .b(new_n254), .out0(new_n256));
  aoi112aa1n03x4               g161(.a(new_n252), .b(new_n256), .c(new_n250), .d(new_n253), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n252), .o1(new_n258));
  norb02aa1n02x5               g163(.a(new_n253), .b(new_n252), .out0(new_n259));
  nand02aa1d04x5               g164(.a(new_n250), .b(new_n259), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n256), .o1(new_n261));
  aoi012aa1n06x5               g166(.a(new_n261), .b(new_n260), .c(new_n258), .o1(new_n262));
  nor042aa1n03x5               g167(.a(new_n262), .b(new_n257), .o1(\s[24] ));
  nano23aa1n06x5               g168(.a(new_n252), .b(new_n254), .c(new_n255), .d(new_n253), .out0(new_n264));
  nanb03aa1n02x5               g169(.a(new_n226), .b(new_n264), .c(new_n244), .out0(new_n265));
  nona22aa1n02x4               g170(.a(new_n255), .b(new_n254), .c(new_n252), .out0(new_n266));
  aoi022aa1n09x5               g171(.a(new_n264), .b(new_n248), .c(new_n266), .d(new_n255), .o1(new_n267));
  inv020aa1n03x5               g172(.a(new_n267), .o1(new_n268));
  aoi013aa1n03x5               g173(.a(new_n268), .b(new_n247), .c(new_n244), .d(new_n264), .o1(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n265), .c(new_n190), .d(new_n195), .o1(new_n270));
  xorb03aa1n02x5               g175(.a(new_n270), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n06x5               g176(.a(\b[24] ), .b(\a[25] ), .o1(new_n272));
  nanp02aa1n02x5               g177(.a(\b[24] ), .b(\a[25] ), .o1(new_n273));
  xnrc02aa1n12x5               g178(.a(\b[25] ), .b(\a[26] ), .out0(new_n274));
  inv000aa1d42x5               g179(.a(new_n274), .o1(new_n275));
  aoi112aa1n03x4               g180(.a(new_n272), .b(new_n275), .c(new_n270), .d(new_n273), .o1(new_n276));
  inv000aa1n06x5               g181(.a(new_n272), .o1(new_n277));
  norb02aa1n02x5               g182(.a(new_n273), .b(new_n272), .out0(new_n278));
  nanp02aa1n03x5               g183(.a(new_n270), .b(new_n278), .o1(new_n279));
  tech160nm_fiaoi012aa1n05x5   g184(.a(new_n274), .b(new_n279), .c(new_n277), .o1(new_n280));
  norp02aa1n03x5               g185(.a(new_n280), .b(new_n276), .o1(\s[26] ));
  nanp02aa1n02x5               g186(.a(new_n162), .b(new_n192), .o1(new_n282));
  nona22aa1n03x5               g187(.a(new_n282), .b(new_n194), .c(new_n193), .out0(new_n283));
  nano22aa1d15x5               g188(.a(new_n274), .b(new_n277), .c(new_n273), .out0(new_n284));
  nano22aa1n03x7               g189(.a(new_n245), .b(new_n264), .c(new_n284), .out0(new_n285));
  aoai13aa1n06x5               g190(.a(new_n285), .b(new_n283), .c(new_n125), .d(new_n189), .o1(new_n286));
  nano22aa1n03x5               g191(.a(new_n229), .b(new_n244), .c(new_n264), .out0(new_n287));
  oaoi03aa1n12x5               g192(.a(\a[26] ), .b(\b[25] ), .c(new_n277), .o1(new_n288));
  oaoi13aa1n12x5               g193(.a(new_n288), .b(new_n284), .c(new_n287), .d(new_n268), .o1(new_n289));
  norp02aa1n02x5               g194(.a(\b[26] ), .b(\a[27] ), .o1(new_n290));
  nanp02aa1n02x5               g195(.a(\b[26] ), .b(\a[27] ), .o1(new_n291));
  norb02aa1n02x5               g196(.a(new_n291), .b(new_n290), .out0(new_n292));
  xnbna2aa1n03x5               g197(.a(new_n292), .b(new_n286), .c(new_n289), .out0(\s[27] ));
  inv000aa1n06x5               g198(.a(new_n290), .o1(new_n294));
  aobi12aa1n02x7               g199(.a(new_n292), .b(new_n286), .c(new_n289), .out0(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[27] ), .b(\a[28] ), .out0(new_n296));
  nano22aa1n02x4               g201(.a(new_n295), .b(new_n294), .c(new_n296), .out0(new_n297));
  nanp03aa1n02x5               g202(.a(new_n247), .b(new_n244), .c(new_n264), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n284), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n288), .o1(new_n300));
  aoai13aa1n09x5               g205(.a(new_n300), .b(new_n299), .c(new_n298), .d(new_n267), .o1(new_n301));
  aoai13aa1n03x5               g206(.a(new_n292), .b(new_n301), .c(new_n196), .d(new_n285), .o1(new_n302));
  tech160nm_fiaoi012aa1n02p5x5 g207(.a(new_n296), .b(new_n302), .c(new_n294), .o1(new_n303));
  norp02aa1n03x5               g208(.a(new_n303), .b(new_n297), .o1(\s[28] ));
  xnrc02aa1n02x5               g209(.a(\b[28] ), .b(\a[29] ), .out0(new_n305));
  nano22aa1n02x4               g210(.a(new_n296), .b(new_n294), .c(new_n291), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n301), .c(new_n196), .d(new_n285), .o1(new_n307));
  oao003aa1n02x5               g212(.a(\a[28] ), .b(\b[27] ), .c(new_n294), .carry(new_n308));
  tech160nm_fiaoi012aa1n02p5x5 g213(.a(new_n305), .b(new_n307), .c(new_n308), .o1(new_n309));
  aobi12aa1n02x7               g214(.a(new_n306), .b(new_n286), .c(new_n289), .out0(new_n310));
  nano22aa1n02x4               g215(.a(new_n310), .b(new_n305), .c(new_n308), .out0(new_n311));
  norp02aa1n03x5               g216(.a(new_n309), .b(new_n311), .o1(\s[29] ));
  xorb03aa1n02x5               g217(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g218(.a(new_n305), .b(new_n296), .c(new_n291), .d(new_n294), .out0(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n301), .c(new_n196), .d(new_n285), .o1(new_n315));
  oao003aa1n02x5               g220(.a(\a[29] ), .b(\b[28] ), .c(new_n308), .carry(new_n316));
  xnrc02aa1n02x5               g221(.a(\b[29] ), .b(\a[30] ), .out0(new_n317));
  tech160nm_fiaoi012aa1n02p5x5 g222(.a(new_n317), .b(new_n315), .c(new_n316), .o1(new_n318));
  aobi12aa1n02x7               g223(.a(new_n314), .b(new_n286), .c(new_n289), .out0(new_n319));
  nano22aa1n03x5               g224(.a(new_n319), .b(new_n316), .c(new_n317), .out0(new_n320));
  norp02aa1n03x5               g225(.a(new_n318), .b(new_n320), .o1(\s[30] ));
  xnrc02aa1n02x5               g226(.a(\b[30] ), .b(\a[31] ), .out0(new_n322));
  norb03aa1n02x5               g227(.a(new_n306), .b(new_n305), .c(new_n317), .out0(new_n323));
  aobi12aa1n02x7               g228(.a(new_n323), .b(new_n286), .c(new_n289), .out0(new_n324));
  oao003aa1n02x5               g229(.a(\a[30] ), .b(\b[29] ), .c(new_n316), .carry(new_n325));
  nano22aa1n03x5               g230(.a(new_n324), .b(new_n322), .c(new_n325), .out0(new_n326));
  aoai13aa1n03x5               g231(.a(new_n323), .b(new_n301), .c(new_n196), .d(new_n285), .o1(new_n327));
  tech160nm_fiaoi012aa1n02p5x5 g232(.a(new_n322), .b(new_n327), .c(new_n325), .o1(new_n328));
  norp02aa1n03x5               g233(.a(new_n328), .b(new_n326), .o1(\s[31] ));
  xnrb03aa1n02x5               g234(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g235(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n331));
  xorb03aa1n02x5               g236(.a(new_n331), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g237(.a(new_n149), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g238(.a(\b[4] ), .b(\a[5] ), .o1(new_n334));
  tech160nm_fioai012aa1n05x5   g239(.a(new_n334), .b(new_n149), .c(new_n122), .o1(new_n335));
  xorb03aa1n02x5               g240(.a(new_n335), .b(\b[5] ), .c(new_n120), .out0(\s[6] ));
  nanp02aa1n02x5               g241(.a(\b[5] ), .b(\a[6] ), .o1(new_n337));
  nanb02aa1n02x5               g242(.a(new_n117), .b(new_n335), .out0(new_n338));
  norb02aa1n02x5               g243(.a(new_n111), .b(new_n114), .out0(new_n339));
  xobna2aa1n03x5               g244(.a(new_n339), .b(new_n338), .c(new_n337), .out0(\s[7] ));
  aoi013aa1n03x5               g245(.a(new_n114), .b(new_n338), .c(new_n339), .d(new_n337), .o1(new_n341));
  xnrb03aa1n02x5               g246(.a(new_n341), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g247(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


