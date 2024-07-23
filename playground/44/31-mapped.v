// Benchmark "adder" written by ABC on Thu Jul 18 10:48:01 2024

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
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n179, new_n180, new_n181,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n322, new_n325, new_n326,
    new_n328, new_n329, new_n330, new_n331, new_n332, new_n333, new_n334,
    new_n336;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor042aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand22aa1n04x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nand22aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  aoi012aa1n06x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  nor022aa1n16x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanp02aa1n04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nor022aa1n16x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand42aa1n03x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nona23aa1n06x5               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  oaih12aa1n02x5               g011(.a(new_n103), .b(new_n104), .c(new_n102), .o1(new_n107));
  oai012aa1n12x5               g012(.a(new_n107), .b(new_n106), .c(new_n101), .o1(new_n108));
  inv040aa1n08x5               g013(.a(\a[5] ), .o1(new_n109));
  inv000aa1d42x5               g014(.a(\a[6] ), .o1(new_n110));
  inv020aa1n20x5               g015(.a(\b[4] ), .o1(new_n111));
  aboi22aa1d24x5               g016(.a(\b[5] ), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n112));
  xnrc02aa1n12x5               g017(.a(\b[7] ), .b(\a[8] ), .out0(new_n113));
  nanp02aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nand42aa1n16x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nor022aa1n16x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nand42aa1n08x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nanb03aa1d24x5               g022(.a(new_n116), .b(new_n117), .c(new_n115), .out0(new_n118));
  nano23aa1d15x5               g023(.a(new_n118), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n119));
  nanp02aa1n06x5               g024(.a(new_n108), .b(new_n119), .o1(new_n120));
  oai022aa1n02x5               g025(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n121));
  aob012aa1n02x5               g026(.a(new_n121), .b(\b[7] ), .c(\a[8] ), .out0(new_n122));
  oai013aa1n09x5               g027(.a(new_n122), .b(new_n118), .c(new_n113), .d(new_n112), .o1(new_n123));
  nanb02aa1n06x5               g028(.a(new_n123), .b(new_n120), .out0(new_n124));
  nand42aa1d28x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  aoi012aa1n02x5               g030(.a(new_n97), .b(new_n124), .c(new_n125), .o1(new_n126));
  xnrb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand42aa1d28x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nor042aa1n06x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nano23aa1n09x5               g034(.a(new_n97), .b(new_n129), .c(new_n128), .d(new_n125), .out0(new_n130));
  oai022aa1d18x5               g035(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n131));
  ao0022aa1n03x7               g036(.a(new_n124), .b(new_n130), .c(new_n131), .d(new_n128), .o(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d32x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand02aa1d28x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nor042aa1d18x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nand02aa1d28x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(new_n138));
  aoai13aa1n03x5               g043(.a(new_n138), .b(new_n134), .c(new_n132), .d(new_n135), .o1(new_n139));
  aoi112aa1n03x4               g044(.a(new_n134), .b(new_n138), .c(new_n132), .d(new_n135), .o1(new_n140));
  nanb02aa1n03x5               g045(.a(new_n140), .b(new_n139), .out0(\s[12] ));
  nano23aa1n09x5               g046(.a(new_n134), .b(new_n136), .c(new_n137), .d(new_n135), .out0(new_n142));
  nand02aa1d06x5               g047(.a(new_n142), .b(new_n130), .o1(new_n143));
  inv000aa1d42x5               g048(.a(new_n143), .o1(new_n144));
  aoai13aa1n06x5               g049(.a(new_n144), .b(new_n123), .c(new_n108), .d(new_n119), .o1(new_n145));
  oai112aa1n03x5               g050(.a(new_n131), .b(new_n128), .c(\b[10] ), .d(\a[11] ), .o1(new_n146));
  nanb03aa1n03x5               g051(.a(new_n136), .b(new_n137), .c(new_n135), .out0(new_n147));
  tech160nm_fioai012aa1n03p5x5 g052(.a(new_n137), .b(new_n136), .c(new_n134), .o1(new_n148));
  oai012aa1n02x5               g053(.a(new_n148), .b(new_n146), .c(new_n147), .o1(new_n149));
  nanb02aa1n02x5               g054(.a(new_n149), .b(new_n145), .out0(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nand02aa1d08x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  tech160nm_fiaoi012aa1n05x5   g058(.a(new_n152), .b(new_n150), .c(new_n153), .o1(new_n154));
  nor002aa1n10x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nand02aa1n08x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n156), .b(new_n155), .out0(new_n157));
  xnrc02aa1n03x5               g062(.a(new_n154), .b(new_n157), .out0(\s[14] ));
  nona23aa1n03x5               g063(.a(new_n156), .b(new_n153), .c(new_n152), .d(new_n155), .out0(new_n159));
  nano23aa1n03x7               g064(.a(new_n152), .b(new_n155), .c(new_n156), .d(new_n153), .out0(new_n160));
  oa0012aa1n02x5               g065(.a(new_n156), .b(new_n155), .c(new_n152), .o(new_n161));
  tech160nm_fiaoi012aa1n02p5x5 g066(.a(new_n161), .b(new_n149), .c(new_n160), .o1(new_n162));
  oai012aa1n03x5               g067(.a(new_n162), .b(new_n145), .c(new_n159), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nand02aa1d08x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  aoi012aa1n03x5               g071(.a(new_n165), .b(new_n163), .c(new_n166), .o1(new_n167));
  xnrb03aa1n03x5               g072(.a(new_n167), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  nor042aa1n04x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nand02aa1n16x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nano23aa1n06x5               g075(.a(new_n165), .b(new_n169), .c(new_n170), .d(new_n166), .out0(new_n171));
  inv000aa1n02x5               g076(.a(new_n171), .o1(new_n172));
  nano22aa1n12x5               g077(.a(new_n143), .b(new_n160), .c(new_n171), .out0(new_n173));
  aoai13aa1n12x5               g078(.a(new_n173), .b(new_n123), .c(new_n108), .d(new_n119), .o1(new_n174));
  oa0012aa1n03x5               g079(.a(new_n170), .b(new_n169), .c(new_n165), .o(new_n175));
  inv000aa1n02x5               g080(.a(new_n175), .o1(new_n176));
  oai112aa1n06x5               g081(.a(new_n174), .b(new_n176), .c(new_n162), .d(new_n172), .o1(new_n177));
  xorb03aa1n03x5               g082(.a(new_n177), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1n02x5               g083(.a(\b[16] ), .b(\a[17] ), .o1(new_n179));
  xorc02aa1n12x5               g084(.a(\a[17] ), .b(\b[16] ), .out0(new_n180));
  tech160nm_fiaoi012aa1n05x5   g085(.a(new_n179), .b(new_n177), .c(new_n180), .o1(new_n181));
  xnrb03aa1n03x5               g086(.a(new_n181), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  oaoi13aa1n02x7               g087(.a(new_n159), .b(new_n148), .c(new_n146), .d(new_n147), .o1(new_n183));
  oaoi13aa1n09x5               g088(.a(new_n175), .b(new_n171), .c(new_n183), .d(new_n161), .o1(new_n184));
  nor042aa1n04x5               g089(.a(\b[17] ), .b(\a[18] ), .o1(new_n185));
  nand02aa1n08x5               g090(.a(\b[17] ), .b(\a[18] ), .o1(new_n186));
  nano22aa1d15x5               g091(.a(new_n185), .b(new_n180), .c(new_n186), .out0(new_n187));
  inv000aa1d42x5               g092(.a(new_n187), .o1(new_n188));
  oa0012aa1n02x5               g093(.a(new_n186), .b(new_n185), .c(new_n179), .o(new_n189));
  inv000aa1d42x5               g094(.a(new_n189), .o1(new_n190));
  aoai13aa1n02x5               g095(.a(new_n190), .b(new_n188), .c(new_n184), .d(new_n174), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g097(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1d30x5               g098(.a(\a[19] ), .o1(new_n194));
  inv040aa1d32x5               g099(.a(\b[18] ), .o1(new_n195));
  nand02aa1d28x5               g100(.a(new_n195), .b(new_n194), .o1(new_n196));
  inv000aa1d42x5               g101(.a(new_n196), .o1(new_n197));
  nand42aa1n10x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nor002aa1d32x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nand42aa1d28x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  norb02aa1n09x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  inv000aa1n02x5               g106(.a(new_n201), .o1(new_n202));
  aoai13aa1n02x5               g107(.a(new_n202), .b(new_n197), .c(new_n191), .d(new_n198), .o1(new_n203));
  oai022aa1d24x5               g108(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n204));
  aoi022aa1n03x5               g109(.a(new_n177), .b(new_n187), .c(new_n186), .d(new_n204), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(new_n196), .b(new_n198), .o1(new_n206));
  oai112aa1n03x5               g111(.a(new_n201), .b(new_n196), .c(new_n205), .d(new_n206), .o1(new_n207));
  nanp02aa1n03x5               g112(.a(new_n207), .b(new_n203), .o1(\s[20] ));
  nona22aa1n06x5               g113(.a(new_n187), .b(new_n206), .c(new_n202), .out0(new_n209));
  nand23aa1n06x5               g114(.a(new_n204), .b(new_n196), .c(new_n186), .o1(new_n210));
  nanb03aa1d24x5               g115(.a(new_n199), .b(new_n200), .c(new_n198), .out0(new_n211));
  aoai13aa1n09x5               g116(.a(new_n200), .b(new_n199), .c(new_n194), .d(new_n195), .o1(new_n212));
  oai012aa1d24x5               g117(.a(new_n212), .b(new_n210), .c(new_n211), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  aoai13aa1n04x5               g119(.a(new_n214), .b(new_n209), .c(new_n184), .d(new_n174), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xnrc02aa1n12x5               g122(.a(\b[20] ), .b(\a[21] ), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  xnrc02aa1n12x5               g124(.a(\b[21] ), .b(\a[22] ), .out0(new_n220));
  aoai13aa1n03x5               g125(.a(new_n220), .b(new_n217), .c(new_n215), .d(new_n219), .o1(new_n221));
  nand02aa1n02x5               g126(.a(new_n215), .b(new_n219), .o1(new_n222));
  nona22aa1n02x5               g127(.a(new_n222), .b(new_n220), .c(new_n217), .out0(new_n223));
  nanp02aa1n03x5               g128(.a(new_n223), .b(new_n221), .o1(\s[22] ));
  nor042aa1n06x5               g129(.a(new_n220), .b(new_n218), .o1(new_n225));
  norb02aa1n02x7               g130(.a(new_n225), .b(new_n209), .out0(new_n226));
  inv040aa1n03x5               g131(.a(new_n226), .o1(new_n227));
  inv040aa1d32x5               g132(.a(\a[22] ), .o1(new_n228));
  inv040aa1d32x5               g133(.a(\b[21] ), .o1(new_n229));
  oao003aa1n12x5               g134(.a(new_n228), .b(new_n229), .c(new_n217), .carry(new_n230));
  aoi012aa1d18x5               g135(.a(new_n230), .b(new_n213), .c(new_n225), .o1(new_n231));
  aoai13aa1n02x7               g136(.a(new_n231), .b(new_n227), .c(new_n184), .d(new_n174), .o1(new_n232));
  xorb03aa1n02x5               g137(.a(new_n232), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  xorc02aa1n12x5               g139(.a(\a[23] ), .b(\b[22] ), .out0(new_n235));
  tech160nm_fixnrc02aa1n05x5   g140(.a(\b[23] ), .b(\a[24] ), .out0(new_n236));
  aoai13aa1n02x5               g141(.a(new_n236), .b(new_n234), .c(new_n232), .d(new_n235), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n231), .o1(new_n238));
  aoai13aa1n03x5               g143(.a(new_n235), .b(new_n238), .c(new_n177), .d(new_n226), .o1(new_n239));
  nona22aa1n03x5               g144(.a(new_n239), .b(new_n236), .c(new_n234), .out0(new_n240));
  nanp02aa1n03x5               g145(.a(new_n237), .b(new_n240), .o1(\s[24] ));
  oai012aa1n02x5               g146(.a(new_n128), .b(\b[10] ), .c(\a[11] ), .o1(new_n242));
  oab012aa1n02x4               g147(.a(new_n242), .b(new_n97), .c(new_n129), .out0(new_n243));
  nano22aa1n02x4               g148(.a(new_n136), .b(new_n135), .c(new_n137), .out0(new_n244));
  inv000aa1n02x5               g149(.a(new_n148), .o1(new_n245));
  aoai13aa1n02x5               g150(.a(new_n160), .b(new_n245), .c(new_n243), .d(new_n244), .o1(new_n246));
  inv000aa1n02x5               g151(.a(new_n161), .o1(new_n247));
  aoai13aa1n03x5               g152(.a(new_n176), .b(new_n172), .c(new_n246), .d(new_n247), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n235), .b(new_n236), .out0(new_n249));
  nano22aa1n03x7               g154(.a(new_n209), .b(new_n225), .c(new_n249), .out0(new_n250));
  aoai13aa1n03x5               g155(.a(new_n250), .b(new_n248), .c(new_n124), .d(new_n173), .o1(new_n251));
  oai012aa1n02x5               g156(.a(new_n186), .b(\b[18] ), .c(\a[19] ), .o1(new_n252));
  oab012aa1n04x5               g157(.a(new_n252), .b(new_n179), .c(new_n185), .out0(new_n253));
  nano22aa1n02x4               g158(.a(new_n199), .b(new_n198), .c(new_n200), .out0(new_n254));
  inv030aa1n02x5               g159(.a(new_n212), .o1(new_n255));
  aoai13aa1n06x5               g160(.a(new_n225), .b(new_n255), .c(new_n253), .d(new_n254), .o1(new_n256));
  inv000aa1n02x5               g161(.a(new_n230), .o1(new_n257));
  inv000aa1n02x5               g162(.a(new_n249), .o1(new_n258));
  oai022aa1n02x5               g163(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n259));
  aob012aa1n02x5               g164(.a(new_n259), .b(\b[23] ), .c(\a[24] ), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n258), .c(new_n256), .d(new_n257), .o1(new_n261));
  inv000aa1n02x5               g166(.a(new_n261), .o1(new_n262));
  xorc02aa1n12x5               g167(.a(\a[25] ), .b(\b[24] ), .out0(new_n263));
  xnbna2aa1n03x5               g168(.a(new_n263), .b(new_n251), .c(new_n262), .out0(\s[25] ));
  nand02aa1n02x5               g169(.a(new_n251), .b(new_n262), .o1(new_n265));
  norp02aa1n02x5               g170(.a(\b[24] ), .b(\a[25] ), .o1(new_n266));
  norp02aa1n04x5               g171(.a(\b[25] ), .b(\a[26] ), .o1(new_n267));
  nand02aa1n04x5               g172(.a(\b[25] ), .b(\a[26] ), .o1(new_n268));
  nanb02aa1n12x5               g173(.a(new_n267), .b(new_n268), .out0(new_n269));
  aoai13aa1n02x5               g174(.a(new_n269), .b(new_n266), .c(new_n265), .d(new_n263), .o1(new_n270));
  aoai13aa1n03x5               g175(.a(new_n263), .b(new_n261), .c(new_n177), .d(new_n250), .o1(new_n271));
  nona22aa1n03x5               g176(.a(new_n271), .b(new_n269), .c(new_n266), .out0(new_n272));
  nanp02aa1n03x5               g177(.a(new_n270), .b(new_n272), .o1(\s[26] ));
  norb02aa1n06x5               g178(.a(new_n263), .b(new_n269), .out0(new_n274));
  nano32aa1n03x7               g179(.a(new_n209), .b(new_n274), .c(new_n225), .d(new_n249), .out0(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n248), .c(new_n124), .d(new_n173), .o1(new_n276));
  oai012aa1n02x5               g181(.a(new_n268), .b(new_n267), .c(new_n266), .o1(new_n277));
  aobi12aa1n06x5               g182(.a(new_n277), .b(new_n261), .c(new_n274), .out0(new_n278));
  xorc02aa1n12x5               g183(.a(\a[27] ), .b(\b[26] ), .out0(new_n279));
  xnbna2aa1n03x5               g184(.a(new_n279), .b(new_n276), .c(new_n278), .out0(\s[27] ));
  nand42aa1n02x5               g185(.a(new_n276), .b(new_n278), .o1(new_n281));
  norp02aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  xorc02aa1n12x5               g187(.a(\a[28] ), .b(\b[27] ), .out0(new_n283));
  inv000aa1d42x5               g188(.a(new_n283), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n284), .b(new_n282), .c(new_n281), .d(new_n279), .o1(new_n285));
  aoai13aa1n02x5               g190(.a(new_n249), .b(new_n230), .c(new_n213), .d(new_n225), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n274), .o1(new_n287));
  aoai13aa1n03x5               g192(.a(new_n277), .b(new_n287), .c(new_n286), .d(new_n260), .o1(new_n288));
  aoai13aa1n03x5               g193(.a(new_n279), .b(new_n288), .c(new_n177), .d(new_n275), .o1(new_n289));
  nona22aa1n03x5               g194(.a(new_n289), .b(new_n284), .c(new_n282), .out0(new_n290));
  nanp02aa1n03x5               g195(.a(new_n285), .b(new_n290), .o1(\s[28] ));
  and002aa1n09x5               g196(.a(new_n283), .b(new_n279), .o(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n288), .c(new_n177), .d(new_n275), .o1(new_n293));
  oai022aa1n02x5               g198(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n294));
  aob012aa1n02x5               g199(.a(new_n294), .b(\b[27] ), .c(\a[28] ), .out0(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[28] ), .b(\a[29] ), .out0(new_n296));
  aoi012aa1n03x5               g201(.a(new_n296), .b(new_n293), .c(new_n295), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n292), .o1(new_n298));
  aoi012aa1n02x5               g203(.a(new_n298), .b(new_n276), .c(new_n278), .o1(new_n299));
  nano22aa1n02x4               g204(.a(new_n299), .b(new_n295), .c(new_n296), .out0(new_n300));
  nor002aa1n02x5               g205(.a(new_n297), .b(new_n300), .o1(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g207(.a(new_n296), .b(new_n279), .c(new_n283), .out0(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n288), .c(new_n177), .d(new_n275), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[29] ), .b(\b[28] ), .c(new_n295), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[29] ), .b(\a[30] ), .out0(new_n306));
  aoi012aa1n03x5               g211(.a(new_n306), .b(new_n304), .c(new_n305), .o1(new_n307));
  inv000aa1n02x5               g212(.a(new_n303), .o1(new_n308));
  aoi012aa1n02x5               g213(.a(new_n308), .b(new_n276), .c(new_n278), .o1(new_n309));
  nano22aa1n02x4               g214(.a(new_n309), .b(new_n305), .c(new_n306), .out0(new_n310));
  nor002aa1n02x5               g215(.a(new_n307), .b(new_n310), .o1(\s[30] ));
  nano23aa1n06x5               g216(.a(new_n306), .b(new_n296), .c(new_n283), .d(new_n279), .out0(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n288), .c(new_n177), .d(new_n275), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .c(new_n305), .carry(new_n314));
  xnrc02aa1n02x5               g219(.a(\b[30] ), .b(\a[31] ), .out0(new_n315));
  aoi012aa1n03x5               g220(.a(new_n315), .b(new_n313), .c(new_n314), .o1(new_n316));
  inv000aa1n02x5               g221(.a(new_n312), .o1(new_n317));
  tech160nm_fiaoi012aa1n02p5x5 g222(.a(new_n317), .b(new_n276), .c(new_n278), .o1(new_n318));
  nano22aa1n02x4               g223(.a(new_n318), .b(new_n314), .c(new_n315), .out0(new_n319));
  norp02aa1n03x5               g224(.a(new_n316), .b(new_n319), .o1(\s[31] ));
  xnrb03aa1n02x5               g225(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g226(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g228(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g229(.a(\b[5] ), .b(new_n110), .out0(new_n325));
  oaoi03aa1n02x5               g230(.a(new_n109), .b(new_n111), .c(new_n108), .o1(new_n326));
  xnbna2aa1n03x5               g231(.a(new_n326), .b(new_n325), .c(new_n115), .out0(\s[6] ));
  nano22aa1n02x4               g232(.a(new_n326), .b(new_n325), .c(new_n115), .out0(new_n328));
  inv000aa1d42x5               g233(.a(new_n112), .o1(new_n329));
  inv000aa1d42x5               g234(.a(new_n118), .o1(new_n330));
  xnrc02aa1n02x5               g235(.a(\b[4] ), .b(\a[5] ), .out0(new_n331));
  oaoi13aa1n02x5               g236(.a(new_n331), .b(new_n107), .c(new_n106), .d(new_n101), .o1(new_n332));
  oai012aa1n02x5               g237(.a(new_n330), .b(new_n332), .c(new_n329), .o1(new_n333));
  oaib12aa1n02x5               g238(.a(new_n325), .b(new_n116), .c(new_n117), .out0(new_n334));
  oa0012aa1n02x5               g239(.a(new_n333), .b(new_n328), .c(new_n334), .o(\s[7] ));
  orn002aa1n02x5               g240(.a(\a[7] ), .b(\b[6] ), .o(new_n336));
  xobna2aa1n03x5               g241(.a(new_n113), .b(new_n333), .c(new_n336), .out0(\s[8] ));
  xorb03aa1n02x5               g242(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


