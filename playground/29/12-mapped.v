// Benchmark "adder" written by ABC on Thu Jul 18 02:54:10 2024

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
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n320, new_n321, new_n323, new_n325, new_n327,
    new_n329, new_n330, new_n331;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d32x5               g001(.a(\a[9] ), .o1(new_n97));
  inv030aa1d32x5               g002(.a(\b[8] ), .o1(new_n98));
  nand42aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  orn002aa1n03x5               g004(.a(\a[2] ), .b(\b[1] ), .o(new_n100));
  nanp02aa1n04x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nand02aa1d24x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  aob012aa1n06x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .out0(new_n103));
  nor042aa1n03x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nanp02aa1n09x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norb02aa1n06x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nor042aa1d18x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  norb02aa1n06x5               g013(.a(new_n108), .b(new_n107), .out0(new_n109));
  nand23aa1n06x5               g014(.a(new_n103), .b(new_n106), .c(new_n109), .o1(new_n110));
  tech160nm_fioai012aa1n03p5x5 g015(.a(new_n105), .b(new_n107), .c(new_n104), .o1(new_n111));
  nor002aa1n08x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nand22aa1n12x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nor022aa1n16x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nand02aa1d08x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nano23aa1n03x7               g020(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n116));
  nor002aa1d32x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nand42aa1d28x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nor042aa1n04x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nand42aa1n10x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  nano23aa1n09x5               g025(.a(new_n117), .b(new_n119), .c(new_n120), .d(new_n118), .out0(new_n121));
  nand22aa1n03x5               g026(.a(new_n121), .b(new_n116), .o1(new_n122));
  and002aa1n12x5               g027(.a(\b[5] ), .b(\a[6] ), .o(new_n123));
  oab012aa1n09x5               g028(.a(new_n123), .b(new_n112), .c(new_n114), .out0(new_n124));
  inv040aa1n06x5               g029(.a(new_n117), .o1(new_n125));
  oai112aa1n06x5               g030(.a(new_n125), .b(new_n118), .c(\b[6] ), .d(\a[7] ), .o1(new_n126));
  aoi022aa1n06x5               g031(.a(new_n121), .b(new_n124), .c(new_n118), .d(new_n126), .o1(new_n127));
  aoai13aa1n12x5               g032(.a(new_n127), .b(new_n122), .c(new_n110), .d(new_n111), .o1(new_n128));
  tech160nm_fixorc02aa1n03p5x5 g033(.a(\a[9] ), .b(\b[8] ), .out0(new_n129));
  nanp02aa1n02x5               g034(.a(new_n128), .b(new_n129), .o1(new_n130));
  tech160nm_fixorc02aa1n03p5x5 g035(.a(\a[10] ), .b(\b[9] ), .out0(new_n131));
  and002aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o(new_n132));
  oai022aa1n02x5               g037(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n133));
  nona22aa1n02x4               g038(.a(new_n130), .b(new_n132), .c(new_n133), .out0(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n131), .c(new_n99), .d(new_n130), .o1(\s[10] ));
  and002aa1n02x5               g040(.a(new_n131), .b(new_n129), .o(new_n136));
  oaoi03aa1n09x5               g041(.a(\a[10] ), .b(\b[9] ), .c(new_n99), .o1(new_n137));
  tech160nm_fiaoi012aa1n05x5   g042(.a(new_n137), .b(new_n128), .c(new_n136), .o1(new_n138));
  nor002aa1d24x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  inv000aa1d42x5               g044(.a(new_n139), .o1(new_n140));
  nand02aa1d12x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n138), .b(new_n141), .c(new_n140), .out0(\s[11] ));
  nanb03aa1n03x5               g047(.a(new_n138), .b(new_n141), .c(new_n140), .out0(new_n143));
  nor042aa1n04x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand02aa1d12x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  nona23aa1n03x5               g051(.a(new_n143), .b(new_n145), .c(new_n144), .d(new_n139), .out0(new_n147));
  aoai13aa1n03x5               g052(.a(new_n147), .b(new_n146), .c(new_n143), .d(new_n140), .o1(\s[12] ));
  nano23aa1n09x5               g053(.a(new_n139), .b(new_n144), .c(new_n145), .d(new_n141), .out0(new_n149));
  and003aa1n02x5               g054(.a(new_n149), .b(new_n131), .c(new_n129), .o(new_n150));
  oai012aa1n02x5               g055(.a(new_n145), .b(new_n144), .c(new_n139), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(new_n149), .b(new_n137), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(new_n152), .b(new_n151), .o1(new_n153));
  ao0012aa1n03x7               g058(.a(new_n153), .b(new_n128), .c(new_n150), .o(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  nand02aa1d04x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanp03aa1n03x5               g063(.a(new_n154), .b(new_n157), .c(new_n158), .o1(new_n159));
  nor042aa1n04x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nand02aa1n08x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  nona23aa1n03x5               g067(.a(new_n159), .b(new_n161), .c(new_n160), .d(new_n156), .out0(new_n163));
  aoai13aa1n02x5               g068(.a(new_n163), .b(new_n162), .c(new_n157), .d(new_n159), .o1(\s[14] ));
  nano23aa1n06x5               g069(.a(new_n156), .b(new_n160), .c(new_n161), .d(new_n158), .out0(new_n165));
  aoai13aa1n06x5               g070(.a(new_n165), .b(new_n153), .c(new_n128), .d(new_n150), .o1(new_n166));
  tech160nm_fioai012aa1n04x5   g071(.a(new_n161), .b(new_n160), .c(new_n156), .o1(new_n167));
  nor002aa1n16x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nand02aa1n06x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  xobna2aa1n03x5               g075(.a(new_n170), .b(new_n166), .c(new_n167), .out0(\s[15] ));
  inv000aa1d42x5               g076(.a(new_n168), .o1(new_n172));
  tech160nm_fiao0012aa1n02p5x5 g077(.a(new_n170), .b(new_n166), .c(new_n167), .o(new_n173));
  nor042aa1n02x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nand02aa1n04x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  norb03aa1n02x5               g081(.a(new_n175), .b(new_n168), .c(new_n174), .out0(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n170), .c(new_n166), .d(new_n167), .o1(new_n178));
  aoai13aa1n03x5               g083(.a(new_n178), .b(new_n176), .c(new_n173), .d(new_n172), .o1(\s[16] ));
  nano23aa1n06x5               g084(.a(new_n168), .b(new_n174), .c(new_n175), .d(new_n169), .out0(new_n180));
  nand22aa1n03x5               g085(.a(new_n180), .b(new_n165), .o1(new_n181));
  nano32aa1n09x5               g086(.a(new_n181), .b(new_n149), .c(new_n131), .d(new_n129), .out0(new_n182));
  nanp02aa1n02x5               g087(.a(new_n128), .b(new_n182), .o1(new_n183));
  oaoi03aa1n02x5               g088(.a(\a[16] ), .b(\b[15] ), .c(new_n172), .o1(new_n184));
  aoib12aa1n06x5               g089(.a(new_n184), .b(new_n180), .c(new_n167), .out0(new_n185));
  aoai13aa1n12x5               g090(.a(new_n185), .b(new_n181), .c(new_n152), .d(new_n151), .o1(new_n186));
  nanb02aa1n02x5               g091(.a(new_n186), .b(new_n183), .out0(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1d18x5               g093(.a(\b[16] ), .b(\a[17] ), .o1(new_n189));
  inv040aa1n08x5               g094(.a(new_n189), .o1(new_n190));
  tech160nm_fixorc02aa1n05x5   g095(.a(\a[17] ), .b(\b[16] ), .out0(new_n191));
  aoai13aa1n02x5               g096(.a(new_n191), .b(new_n186), .c(new_n128), .d(new_n182), .o1(new_n192));
  xorc02aa1n12x5               g097(.a(\a[18] ), .b(\b[17] ), .out0(new_n193));
  inv000aa1d42x5               g098(.a(\a[18] ), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\b[17] ), .o1(new_n195));
  aoi012aa1n02x5               g100(.a(new_n189), .b(new_n194), .c(new_n195), .o1(new_n196));
  oai112aa1n02x5               g101(.a(new_n192), .b(new_n196), .c(new_n195), .d(new_n194), .o1(new_n197));
  aoai13aa1n02x5               g102(.a(new_n197), .b(new_n193), .c(new_n190), .d(new_n192), .o1(\s[18] ));
  and002aa1n02x5               g103(.a(new_n193), .b(new_n191), .o(new_n199));
  aoai13aa1n06x5               g104(.a(new_n199), .b(new_n186), .c(new_n128), .d(new_n182), .o1(new_n200));
  oaoi03aa1n12x5               g105(.a(\a[18] ), .b(\b[17] ), .c(new_n190), .o1(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  nor042aa1n06x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nand02aa1n08x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nanb02aa1n02x5               g109(.a(new_n203), .b(new_n204), .out0(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  xnbna2aa1n03x5               g111(.a(new_n206), .b(new_n200), .c(new_n202), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1n02x5               g113(.a(new_n203), .o1(new_n209));
  aob012aa1n02x5               g114(.a(new_n206), .b(new_n200), .c(new_n202), .out0(new_n210));
  nor042aa1n04x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nanp02aa1n12x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  norb02aa1n02x5               g117(.a(new_n212), .b(new_n211), .out0(new_n213));
  norb03aa1n02x5               g118(.a(new_n212), .b(new_n203), .c(new_n211), .out0(new_n214));
  aoai13aa1n02x5               g119(.a(new_n214), .b(new_n205), .c(new_n200), .d(new_n202), .o1(new_n215));
  aoai13aa1n03x5               g120(.a(new_n215), .b(new_n213), .c(new_n210), .d(new_n209), .o1(\s[20] ));
  nano23aa1n06x5               g121(.a(new_n203), .b(new_n211), .c(new_n212), .d(new_n204), .out0(new_n217));
  nand23aa1n06x5               g122(.a(new_n217), .b(new_n191), .c(new_n193), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  aoai13aa1n04x5               g124(.a(new_n219), .b(new_n186), .c(new_n128), .d(new_n182), .o1(new_n220));
  oaoi03aa1n02x5               g125(.a(\a[20] ), .b(\b[19] ), .c(new_n209), .o1(new_n221));
  aoi012aa1n02x5               g126(.a(new_n221), .b(new_n217), .c(new_n201), .o1(new_n222));
  xorc02aa1n02x5               g127(.a(\a[21] ), .b(\b[20] ), .out0(new_n223));
  xnbna2aa1n03x5               g128(.a(new_n223), .b(new_n220), .c(new_n222), .out0(\s[21] ));
  nor042aa1n06x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aob012aa1n03x5               g131(.a(new_n223), .b(new_n220), .c(new_n222), .out0(new_n227));
  xorc02aa1n02x5               g132(.a(\a[22] ), .b(\b[21] ), .out0(new_n228));
  nanp02aa1n02x5               g133(.a(\b[21] ), .b(\a[22] ), .o1(new_n229));
  oai022aa1n02x5               g134(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n230));
  norb02aa1n02x5               g135(.a(new_n229), .b(new_n230), .out0(new_n231));
  nanp02aa1n03x5               g136(.a(new_n227), .b(new_n231), .o1(new_n232));
  aoai13aa1n03x5               g137(.a(new_n232), .b(new_n228), .c(new_n226), .d(new_n227), .o1(\s[22] ));
  nanp02aa1n02x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  nor042aa1n02x5               g139(.a(\b[21] ), .b(\a[22] ), .o1(new_n235));
  nano23aa1n06x5               g140(.a(new_n225), .b(new_n235), .c(new_n229), .d(new_n234), .out0(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n218), .out0(new_n237));
  aoai13aa1n06x5               g142(.a(new_n237), .b(new_n186), .c(new_n128), .d(new_n182), .o1(new_n238));
  aoai13aa1n06x5               g143(.a(new_n236), .b(new_n221), .c(new_n217), .d(new_n201), .o1(new_n239));
  oai012aa1n12x5               g144(.a(new_n229), .b(new_n235), .c(new_n225), .o1(new_n240));
  nanp02aa1n02x5               g145(.a(new_n239), .b(new_n240), .o1(new_n241));
  inv000aa1n02x5               g146(.a(new_n241), .o1(new_n242));
  xnrc02aa1n12x5               g147(.a(\b[22] ), .b(\a[23] ), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  xnbna2aa1n03x5               g149(.a(new_n244), .b(new_n238), .c(new_n242), .out0(\s[23] ));
  norp02aa1n02x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  aob012aa1n02x5               g152(.a(new_n244), .b(new_n238), .c(new_n242), .out0(new_n248));
  tech160nm_fixorc02aa1n02p5x5 g153(.a(\a[24] ), .b(\b[23] ), .out0(new_n249));
  nanp02aa1n02x5               g154(.a(\b[23] ), .b(\a[24] ), .o1(new_n250));
  oai022aa1n02x5               g155(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n250), .b(new_n251), .out0(new_n252));
  aoai13aa1n02x5               g157(.a(new_n252), .b(new_n243), .c(new_n238), .d(new_n242), .o1(new_n253));
  aoai13aa1n03x5               g158(.a(new_n253), .b(new_n249), .c(new_n248), .d(new_n247), .o1(\s[24] ));
  nano32aa1n02x4               g159(.a(new_n218), .b(new_n249), .c(new_n236), .d(new_n244), .out0(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n186), .c(new_n128), .d(new_n182), .o1(new_n256));
  norb02aa1n02x7               g161(.a(new_n249), .b(new_n243), .out0(new_n257));
  inv000aa1n02x5               g162(.a(new_n257), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(new_n251), .b(new_n250), .o1(new_n259));
  aoai13aa1n12x5               g164(.a(new_n259), .b(new_n258), .c(new_n239), .d(new_n240), .o1(new_n260));
  inv000aa1n02x5               g165(.a(new_n260), .o1(new_n261));
  xorc02aa1n06x5               g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  xnbna2aa1n03x5               g167(.a(new_n262), .b(new_n256), .c(new_n261), .out0(\s[25] ));
  norp02aa1n02x5               g168(.a(\b[24] ), .b(\a[25] ), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  aob012aa1n03x5               g170(.a(new_n262), .b(new_n256), .c(new_n261), .out0(new_n266));
  tech160nm_fixorc02aa1n02p5x5 g171(.a(\a[26] ), .b(\b[25] ), .out0(new_n267));
  nanp02aa1n02x5               g172(.a(\b[25] ), .b(\a[26] ), .o1(new_n268));
  oai022aa1n02x5               g173(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n269));
  norb02aa1n02x5               g174(.a(new_n268), .b(new_n269), .out0(new_n270));
  nand42aa1n02x5               g175(.a(new_n266), .b(new_n270), .o1(new_n271));
  aoai13aa1n02x5               g176(.a(new_n271), .b(new_n267), .c(new_n265), .d(new_n266), .o1(\s[26] ));
  and002aa1n02x7               g177(.a(new_n267), .b(new_n262), .o(new_n273));
  nano32aa1n03x7               g178(.a(new_n218), .b(new_n273), .c(new_n236), .d(new_n257), .out0(new_n274));
  aoai13aa1n12x5               g179(.a(new_n274), .b(new_n186), .c(new_n128), .d(new_n182), .o1(new_n275));
  aoi022aa1n09x5               g180(.a(new_n260), .b(new_n273), .c(new_n268), .d(new_n269), .o1(new_n276));
  xorc02aa1n12x5               g181(.a(\a[27] ), .b(\b[26] ), .out0(new_n277));
  xnbna2aa1n03x5               g182(.a(new_n277), .b(new_n276), .c(new_n275), .out0(\s[27] ));
  norp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n279), .o1(new_n280));
  nand22aa1n03x5               g185(.a(new_n260), .b(new_n273), .o1(new_n281));
  nanp02aa1n02x5               g186(.a(new_n269), .b(new_n268), .o1(new_n282));
  nand23aa1n06x5               g187(.a(new_n281), .b(new_n275), .c(new_n282), .o1(new_n283));
  nanp02aa1n03x5               g188(.a(new_n283), .b(new_n277), .o1(new_n284));
  xorc02aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .out0(new_n285));
  inv000aa1d42x5               g190(.a(new_n277), .o1(new_n286));
  oai022aa1n02x5               g191(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n287));
  aoi012aa1n02x5               g192(.a(new_n287), .b(\a[28] ), .c(\b[27] ), .o1(new_n288));
  aoai13aa1n03x5               g193(.a(new_n288), .b(new_n286), .c(new_n276), .d(new_n275), .o1(new_n289));
  aoai13aa1n03x5               g194(.a(new_n289), .b(new_n285), .c(new_n284), .d(new_n280), .o1(\s[28] ));
  and002aa1n02x5               g195(.a(new_n285), .b(new_n277), .o(new_n291));
  inv000aa1d42x5               g196(.a(new_n291), .o1(new_n292));
  inv000aa1d42x5               g197(.a(\b[27] ), .o1(new_n293));
  oaib12aa1n09x5               g198(.a(new_n287), .b(new_n293), .c(\a[28] ), .out0(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[28] ), .b(\a[29] ), .out0(new_n295));
  norb02aa1n02x5               g200(.a(new_n294), .b(new_n295), .out0(new_n296));
  aoai13aa1n02x5               g201(.a(new_n296), .b(new_n292), .c(new_n276), .d(new_n275), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n294), .o1(new_n298));
  aoai13aa1n03x5               g203(.a(new_n295), .b(new_n298), .c(new_n283), .d(new_n291), .o1(new_n299));
  nanp02aa1n03x5               g204(.a(new_n299), .b(new_n297), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g206(.a(new_n295), .b(new_n277), .c(new_n285), .out0(new_n302));
  oaoi03aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .c(new_n294), .o1(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[29] ), .b(\a[30] ), .out0(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n303), .c(new_n283), .d(new_n302), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n302), .o1(new_n306));
  norp02aa1n02x5               g211(.a(new_n303), .b(new_n304), .o1(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n306), .c(new_n276), .d(new_n275), .o1(new_n308));
  nanp02aa1n03x5               g213(.a(new_n305), .b(new_n308), .o1(\s[30] ));
  nano23aa1n02x4               g214(.a(new_n304), .b(new_n295), .c(new_n285), .d(new_n277), .out0(new_n310));
  nanp02aa1n03x5               g215(.a(new_n283), .b(new_n310), .o1(new_n311));
  xorc02aa1n02x5               g216(.a(\a[31] ), .b(\b[30] ), .out0(new_n312));
  inv000aa1n02x5               g217(.a(new_n310), .o1(new_n313));
  oai012aa1n02x5               g218(.a(new_n312), .b(\b[29] ), .c(\a[30] ), .o1(new_n314));
  aoib12aa1n02x5               g219(.a(new_n314), .b(new_n303), .c(new_n304), .out0(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n313), .c(new_n276), .d(new_n275), .o1(new_n316));
  oao003aa1n02x5               g221(.a(\a[30] ), .b(\b[29] ), .c(new_n307), .carry(new_n317));
  aoai13aa1n03x5               g222(.a(new_n316), .b(new_n312), .c(new_n311), .d(new_n317), .o1(\s[31] ));
  xorb03aa1n02x5               g223(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  inv000aa1d42x5               g224(.a(new_n107), .o1(new_n320));
  nanp02aa1n02x5               g225(.a(new_n103), .b(new_n109), .o1(new_n321));
  xnbna2aa1n03x5               g226(.a(new_n106), .b(new_n321), .c(new_n320), .out0(\s[4] ));
  nanp02aa1n02x5               g227(.a(new_n110), .b(new_n111), .o1(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g229(.a(new_n114), .b(new_n323), .c(new_n115), .o1(new_n325));
  xnrb03aa1n02x5               g230(.a(new_n325), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g231(.a(new_n124), .b(new_n323), .c(new_n116), .o(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  norb02aa1n02x5               g233(.a(new_n120), .b(new_n119), .out0(new_n329));
  nanb02aa1n02x5               g234(.a(new_n117), .b(new_n118), .out0(new_n330));
  aoai13aa1n02x5               g235(.a(new_n330), .b(new_n119), .c(new_n327), .d(new_n120), .o1(new_n331));
  aoai13aa1n02x5               g236(.a(new_n331), .b(new_n126), .c(new_n329), .d(new_n327), .o1(\s[8] ));
  xorb03aa1n02x5               g237(.a(new_n128), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


