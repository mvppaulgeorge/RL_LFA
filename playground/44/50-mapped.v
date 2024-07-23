// Benchmark "adder" written by ABC on Thu Jul 18 11:00:14 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n124, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n159, new_n160, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n181,
    new_n182, new_n183, new_n184, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n330, new_n333,
    new_n335, new_n336, new_n337, new_n338, new_n339, new_n341;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nanp02aa1n04x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  aoi012aa1n12x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n02x4               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  aoi012aa1n02x5               g010(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n106));
  tech160nm_fioai012aa1n04x5   g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  norp02aa1n02x5               g014(.a(new_n109), .b(new_n108), .o1(new_n110));
  xnrc02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .out0(new_n111));
  nand22aa1n03x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nor002aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanb03aa1n03x5               g019(.a(new_n113), .b(new_n114), .c(new_n112), .out0(new_n115));
  nanp02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nano23aa1n06x5               g021(.a(new_n115), .b(new_n111), .c(new_n110), .d(new_n116), .out0(new_n117));
  oai022aa1n02x5               g022(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n118));
  aoi013aa1n02x4               g023(.a(new_n113), .b(new_n118), .c(new_n112), .d(new_n114), .o1(new_n119));
  oaoi03aa1n02x5               g024(.a(\a[8] ), .b(\b[7] ), .c(new_n119), .o1(new_n120));
  tech160nm_fiaoi012aa1n05x5   g025(.a(new_n120), .b(new_n107), .c(new_n117), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[9] ), .b(\b[8] ), .c(new_n121), .o1(new_n122));
  xorb03aa1n02x5               g027(.a(new_n122), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  norp02aa1n02x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand42aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  oai012aa1n02x5               g031(.a(new_n126), .b(new_n125), .c(new_n124), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nano23aa1n03x7               g033(.a(new_n124), .b(new_n125), .c(new_n126), .d(new_n128), .out0(new_n129));
  oaib12aa1n06x5               g034(.a(new_n127), .b(new_n121), .c(new_n129), .out0(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  inv000aa1d42x5               g036(.a(\a[11] ), .o1(new_n132));
  inv000aa1d42x5               g037(.a(\b[10] ), .o1(new_n133));
  oaoi03aa1n02x5               g038(.a(new_n132), .b(new_n133), .c(new_n130), .o1(new_n134));
  xnrb03aa1n03x5               g039(.a(new_n134), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  xorc02aa1n02x5               g040(.a(\a[11] ), .b(\b[10] ), .out0(new_n136));
  norp02aa1n02x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(new_n139));
  nano22aa1n02x4               g044(.a(new_n139), .b(new_n129), .c(new_n136), .out0(new_n140));
  aoai13aa1n02x5               g045(.a(new_n140), .b(new_n120), .c(new_n107), .d(new_n117), .o1(new_n141));
  aoi012aa1n02x5               g046(.a(new_n137), .b(new_n132), .c(new_n133), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(new_n127), .b(new_n142), .o1(new_n143));
  aob012aa1n02x5               g048(.a(new_n138), .b(\b[10] ), .c(\a[11] ), .out0(new_n144));
  oai012aa1n02x5               g049(.a(new_n144), .b(\b[11] ), .c(\a[12] ), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(new_n143), .b(new_n145), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(new_n141), .b(new_n146), .o1(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n06x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  aoi012aa1n02x5               g055(.a(new_n149), .b(new_n147), .c(new_n150), .o1(new_n151));
  xnrb03aa1n02x5               g056(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n04x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nand22aa1n03x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  tech160nm_fiaoi012aa1n04x5   g059(.a(new_n153), .b(new_n149), .c(new_n154), .o1(new_n155));
  nona23aa1n02x4               g060(.a(new_n154), .b(new_n150), .c(new_n149), .d(new_n153), .out0(new_n156));
  aoai13aa1n04x5               g061(.a(new_n155), .b(new_n156), .c(new_n141), .d(new_n146), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  xnrc02aa1n12x5               g064(.a(\b[14] ), .b(\a[15] ), .out0(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  orn002aa1n12x5               g066(.a(\a[16] ), .b(\b[15] ), .o(new_n162));
  nand22aa1n09x5               g067(.a(\b[15] ), .b(\a[16] ), .o1(new_n163));
  nand02aa1d08x5               g068(.a(new_n162), .b(new_n163), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  aoi112aa1n02x5               g070(.a(new_n165), .b(new_n159), .c(new_n157), .d(new_n161), .o1(new_n166));
  aoai13aa1n02x5               g071(.a(new_n165), .b(new_n159), .c(new_n157), .d(new_n161), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(\s[16] ));
  nanb03aa1n02x5               g073(.a(new_n139), .b(new_n129), .c(new_n136), .out0(new_n169));
  nano23aa1n02x4               g074(.a(new_n149), .b(new_n153), .c(new_n154), .d(new_n150), .out0(new_n170));
  nor042aa1n09x5               g075(.a(new_n160), .b(new_n164), .o1(new_n171));
  nano22aa1n03x7               g076(.a(new_n169), .b(new_n170), .c(new_n171), .out0(new_n172));
  aoai13aa1n06x5               g077(.a(new_n172), .b(new_n120), .c(new_n107), .d(new_n117), .o1(new_n173));
  inv000aa1n02x5               g078(.a(new_n155), .o1(new_n174));
  oaoi13aa1n02x5               g079(.a(new_n137), .b(new_n138), .c(new_n132), .d(new_n133), .o1(new_n175));
  aoi112aa1n03x5               g080(.a(new_n156), .b(new_n175), .c(new_n127), .d(new_n142), .o1(new_n176));
  aob012aa1d15x5               g081(.a(new_n162), .b(new_n159), .c(new_n163), .out0(new_n177));
  oaoi13aa1n12x5               g082(.a(new_n177), .b(new_n171), .c(new_n176), .d(new_n174), .o1(new_n178));
  tech160nm_fixorc02aa1n05x5   g083(.a(\a[17] ), .b(\b[16] ), .out0(new_n179));
  xnbna2aa1n03x5               g084(.a(new_n179), .b(new_n173), .c(new_n178), .out0(\s[17] ));
  inv000aa1d42x5               g085(.a(\a[17] ), .o1(new_n181));
  inv000aa1d42x5               g086(.a(\b[16] ), .o1(new_n182));
  norp03aa1n02x5               g087(.a(new_n156), .b(new_n164), .c(new_n160), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(new_n140), .b(new_n183), .o1(new_n184));
  oai012aa1n12x5               g089(.a(new_n178), .b(new_n121), .c(new_n184), .o1(new_n185));
  oaoi03aa1n02x5               g090(.a(new_n181), .b(new_n182), .c(new_n185), .o1(new_n186));
  nor022aa1n16x5               g091(.a(\b[17] ), .b(\a[18] ), .o1(new_n187));
  nand42aa1n08x5               g092(.a(\b[17] ), .b(\a[18] ), .o1(new_n188));
  nanb02aa1n02x5               g093(.a(new_n187), .b(new_n188), .out0(new_n189));
  tech160nm_fixorc02aa1n02p5x5 g094(.a(new_n186), .b(new_n189), .out0(\s[18] ));
  norb02aa1n12x5               g095(.a(new_n179), .b(new_n189), .out0(new_n191));
  inv000aa1d42x5               g096(.a(new_n191), .o1(new_n192));
  aoai13aa1n12x5               g097(.a(new_n188), .b(new_n187), .c(new_n181), .d(new_n182), .o1(new_n193));
  aoai13aa1n02x5               g098(.a(new_n193), .b(new_n192), .c(new_n173), .d(new_n178), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g100(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nand42aa1n08x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nanb02aa1n06x5               g103(.a(new_n197), .b(new_n198), .out0(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  tech160nm_fixnrc02aa1n04x5   g105(.a(\b[19] ), .b(\a[20] ), .out0(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  aoi112aa1n02x5               g107(.a(new_n197), .b(new_n202), .c(new_n194), .d(new_n200), .o1(new_n203));
  inv000aa1d42x5               g108(.a(new_n197), .o1(new_n204));
  inv000aa1d42x5               g109(.a(new_n193), .o1(new_n205));
  aoai13aa1n04x5               g110(.a(new_n200), .b(new_n205), .c(new_n185), .d(new_n191), .o1(new_n206));
  aoi012aa1n02x5               g111(.a(new_n201), .b(new_n206), .c(new_n204), .o1(new_n207));
  norp02aa1n02x5               g112(.a(new_n207), .b(new_n203), .o1(\s[20] ));
  nona23aa1d18x5               g113(.a(new_n202), .b(new_n179), .c(new_n189), .d(new_n199), .out0(new_n209));
  inv000aa1d42x5               g114(.a(\a[20] ), .o1(new_n210));
  inv000aa1d42x5               g115(.a(\b[19] ), .o1(new_n211));
  nanp02aa1n02x5               g116(.a(new_n211), .b(new_n210), .o1(new_n212));
  tech160nm_fiaoi012aa1n04x5   g117(.a(new_n197), .b(new_n210), .c(new_n211), .o1(new_n213));
  aob012aa1n02x5               g118(.a(new_n198), .b(\b[19] ), .c(\a[20] ), .out0(new_n214));
  aoi022aa1n12x5               g119(.a(new_n193), .b(new_n213), .c(new_n214), .d(new_n212), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  aoai13aa1n04x5               g121(.a(new_n216), .b(new_n209), .c(new_n173), .d(new_n178), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  nanp02aa1n02x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  nor042aa1n04x5               g126(.a(\b[21] ), .b(\a[22] ), .o1(new_n222));
  nanp02aa1n04x5               g127(.a(\b[21] ), .b(\a[22] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  aoi112aa1n02x5               g129(.a(new_n219), .b(new_n224), .c(new_n217), .d(new_n221), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n219), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n209), .o1(new_n227));
  aoai13aa1n02x5               g132(.a(new_n221), .b(new_n215), .c(new_n185), .d(new_n227), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n224), .o1(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n228), .c(new_n226), .o1(new_n230));
  norp02aa1n02x5               g135(.a(new_n230), .b(new_n225), .o1(\s[22] ));
  nona23aa1n03x5               g136(.a(new_n223), .b(new_n220), .c(new_n219), .d(new_n222), .out0(new_n232));
  nona32aa1d24x5               g137(.a(new_n191), .b(new_n232), .c(new_n201), .d(new_n199), .out0(new_n233));
  nand02aa1d08x5               g138(.a(new_n193), .b(new_n213), .o1(new_n234));
  nand02aa1d04x5               g139(.a(new_n214), .b(new_n212), .o1(new_n235));
  nano23aa1d12x5               g140(.a(new_n219), .b(new_n222), .c(new_n223), .d(new_n220), .out0(new_n236));
  aoi012aa1d24x5               g141(.a(new_n222), .b(new_n219), .c(new_n223), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  aoi013aa1n09x5               g143(.a(new_n238), .b(new_n234), .c(new_n236), .d(new_n235), .o1(new_n239));
  aoai13aa1n04x5               g144(.a(new_n239), .b(new_n233), .c(new_n173), .d(new_n178), .o1(new_n240));
  xorb03aa1n02x5               g145(.a(new_n240), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n03x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  xorc02aa1n02x5               g147(.a(\a[23] ), .b(\b[22] ), .out0(new_n243));
  xorc02aa1n12x5               g148(.a(\a[24] ), .b(\b[23] ), .out0(new_n244));
  aoi112aa1n02x5               g149(.a(new_n242), .b(new_n244), .c(new_n240), .d(new_n243), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n242), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n233), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n239), .o1(new_n248));
  aoai13aa1n02x5               g153(.a(new_n243), .b(new_n248), .c(new_n185), .d(new_n247), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n244), .o1(new_n250));
  aoi012aa1n02x5               g155(.a(new_n250), .b(new_n249), .c(new_n246), .o1(new_n251));
  norp02aa1n02x5               g156(.a(new_n251), .b(new_n245), .o1(\s[24] ));
  nano32aa1n02x4               g157(.a(new_n209), .b(new_n244), .c(new_n236), .d(new_n243), .out0(new_n253));
  inv030aa1n02x5               g158(.a(new_n253), .o1(new_n254));
  nand23aa1n06x5               g159(.a(new_n234), .b(new_n236), .c(new_n235), .o1(new_n255));
  and002aa1n02x5               g160(.a(new_n244), .b(new_n243), .o(new_n256));
  inv000aa1n02x5               g161(.a(new_n256), .o1(new_n257));
  oao003aa1n02x5               g162(.a(\a[24] ), .b(\b[23] ), .c(new_n246), .carry(new_n258));
  aoai13aa1n12x5               g163(.a(new_n258), .b(new_n257), .c(new_n255), .d(new_n237), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  aoai13aa1n04x5               g165(.a(new_n260), .b(new_n254), .c(new_n173), .d(new_n178), .o1(new_n261));
  xorb03aa1n02x5               g166(.a(new_n261), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g167(.a(\b[24] ), .b(\a[25] ), .o1(new_n263));
  xorc02aa1n02x5               g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  xorc02aa1n12x5               g169(.a(\a[26] ), .b(\b[25] ), .out0(new_n265));
  aoi112aa1n02x5               g170(.a(new_n263), .b(new_n265), .c(new_n261), .d(new_n264), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n263), .o1(new_n267));
  aoai13aa1n03x5               g172(.a(new_n264), .b(new_n259), .c(new_n185), .d(new_n253), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n265), .o1(new_n269));
  aoi012aa1n03x5               g174(.a(new_n269), .b(new_n268), .c(new_n267), .o1(new_n270));
  norp02aa1n02x5               g175(.a(new_n270), .b(new_n266), .o1(\s[26] ));
  inv000aa1d42x5               g176(.a(new_n100), .o1(new_n272));
  nano23aa1n02x4               g177(.a(new_n101), .b(new_n103), .c(new_n104), .d(new_n102), .out0(new_n273));
  aobi12aa1n02x5               g178(.a(new_n106), .b(new_n273), .c(new_n272), .out0(new_n274));
  and002aa1n02x5               g179(.a(\b[7] ), .b(\a[8] ), .o(new_n275));
  norp02aa1n02x5               g180(.a(\b[7] ), .b(\a[8] ), .o1(new_n276));
  nanb02aa1n02x5               g181(.a(new_n113), .b(new_n114), .out0(new_n277));
  nano22aa1n02x4               g182(.a(new_n277), .b(new_n112), .c(new_n116), .out0(new_n278));
  nona32aa1n02x4               g183(.a(new_n278), .b(new_n275), .c(new_n276), .d(new_n118), .out0(new_n279));
  aoi113aa1n02x5               g184(.a(new_n113), .b(new_n276), .c(new_n118), .d(new_n114), .e(new_n112), .o1(new_n280));
  oai022aa1n02x5               g185(.a(new_n274), .b(new_n279), .c(new_n280), .d(new_n275), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n171), .o1(new_n282));
  nanp03aa1n02x5               g187(.a(new_n143), .b(new_n170), .c(new_n145), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n177), .o1(new_n284));
  aoai13aa1n02x5               g189(.a(new_n284), .b(new_n282), .c(new_n283), .d(new_n155), .o1(new_n285));
  and002aa1n06x5               g190(.a(new_n265), .b(new_n264), .o(new_n286));
  nano22aa1d15x5               g191(.a(new_n233), .b(new_n256), .c(new_n286), .out0(new_n287));
  aoai13aa1n06x5               g192(.a(new_n287), .b(new_n285), .c(new_n281), .d(new_n172), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[26] ), .b(\b[25] ), .c(new_n267), .carry(new_n289));
  aobi12aa1n12x5               g194(.a(new_n289), .b(new_n259), .c(new_n286), .out0(new_n290));
  xorc02aa1n02x5               g195(.a(\a[27] ), .b(\b[26] ), .out0(new_n291));
  xnbna2aa1n03x5               g196(.a(new_n291), .b(new_n288), .c(new_n290), .out0(\s[27] ));
  norp02aa1n02x5               g197(.a(\b[26] ), .b(\a[27] ), .o1(new_n293));
  inv040aa1n03x5               g198(.a(new_n293), .o1(new_n294));
  aobi12aa1n03x5               g199(.a(new_n291), .b(new_n288), .c(new_n290), .out0(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[27] ), .b(\a[28] ), .out0(new_n296));
  nano22aa1n03x5               g201(.a(new_n295), .b(new_n294), .c(new_n296), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n256), .b(new_n238), .c(new_n215), .d(new_n236), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n286), .o1(new_n299));
  aoai13aa1n04x5               g204(.a(new_n289), .b(new_n299), .c(new_n298), .d(new_n258), .o1(new_n300));
  aoai13aa1n02x5               g205(.a(new_n291), .b(new_n300), .c(new_n185), .d(new_n287), .o1(new_n301));
  tech160nm_fiaoi012aa1n02p5x5 g206(.a(new_n296), .b(new_n301), .c(new_n294), .o1(new_n302));
  norp02aa1n03x5               g207(.a(new_n302), .b(new_n297), .o1(\s[28] ));
  norb02aa1n02x5               g208(.a(new_n291), .b(new_n296), .out0(new_n304));
  aoai13aa1n02x5               g209(.a(new_n304), .b(new_n300), .c(new_n185), .d(new_n287), .o1(new_n305));
  oao003aa1n02x5               g210(.a(\a[28] ), .b(\b[27] ), .c(new_n294), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[28] ), .b(\a[29] ), .out0(new_n307));
  aoi012aa1n02x5               g212(.a(new_n307), .b(new_n305), .c(new_n306), .o1(new_n308));
  aobi12aa1n06x5               g213(.a(new_n304), .b(new_n288), .c(new_n290), .out0(new_n309));
  nano22aa1n03x7               g214(.a(new_n309), .b(new_n306), .c(new_n307), .out0(new_n310));
  norp02aa1n03x5               g215(.a(new_n308), .b(new_n310), .o1(\s[29] ));
  xorb03aa1n02x5               g216(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g217(.a(new_n291), .b(new_n307), .c(new_n296), .out0(new_n313));
  aoai13aa1n02x5               g218(.a(new_n313), .b(new_n300), .c(new_n185), .d(new_n287), .o1(new_n314));
  oao003aa1n02x5               g219(.a(\a[29] ), .b(\b[28] ), .c(new_n306), .carry(new_n315));
  xnrc02aa1n02x5               g220(.a(\b[29] ), .b(\a[30] ), .out0(new_n316));
  tech160nm_fiaoi012aa1n02p5x5 g221(.a(new_n316), .b(new_n314), .c(new_n315), .o1(new_n317));
  aobi12aa1n03x5               g222(.a(new_n313), .b(new_n288), .c(new_n290), .out0(new_n318));
  nano22aa1n03x5               g223(.a(new_n318), .b(new_n315), .c(new_n316), .out0(new_n319));
  norp02aa1n03x5               g224(.a(new_n317), .b(new_n319), .o1(\s[30] ));
  norb02aa1n02x5               g225(.a(new_n313), .b(new_n316), .out0(new_n321));
  aobi12aa1n06x5               g226(.a(new_n321), .b(new_n288), .c(new_n290), .out0(new_n322));
  oao003aa1n02x5               g227(.a(\a[30] ), .b(\b[29] ), .c(new_n315), .carry(new_n323));
  xnrc02aa1n02x5               g228(.a(\b[30] ), .b(\a[31] ), .out0(new_n324));
  nano22aa1n03x7               g229(.a(new_n322), .b(new_n323), .c(new_n324), .out0(new_n325));
  aoai13aa1n02x5               g230(.a(new_n321), .b(new_n300), .c(new_n185), .d(new_n287), .o1(new_n326));
  aoi012aa1n02x5               g231(.a(new_n324), .b(new_n326), .c(new_n323), .o1(new_n327));
  norp02aa1n03x5               g232(.a(new_n327), .b(new_n325), .o1(\s[31] ));
  xnrb03aa1n02x5               g233(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g234(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n330));
  xorb03aa1n02x5               g235(.a(new_n330), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g236(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g237(.a(\a[5] ), .b(\b[4] ), .c(new_n274), .o1(new_n333));
  xorb03aa1n02x5               g238(.a(new_n333), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g239(.a(\a[6] ), .o1(new_n335));
  nanb02aa1n02x5               g240(.a(\b[5] ), .b(new_n335), .out0(new_n336));
  nanp03aa1n02x5               g241(.a(new_n333), .b(new_n336), .c(new_n112), .o1(new_n337));
  nanb02aa1n02x5               g242(.a(new_n109), .b(new_n116), .out0(new_n338));
  oaoi13aa1n02x5               g243(.a(new_n115), .b(new_n110), .c(new_n274), .d(new_n338), .o1(new_n339));
  aoi013aa1n02x4               g244(.a(new_n339), .b(new_n337), .c(new_n277), .d(new_n336), .o1(\s[7] ));
  norp02aa1n02x5               g245(.a(new_n339), .b(new_n113), .o1(new_n341));
  xnrb03aa1n02x5               g246(.a(new_n341), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g247(.a(new_n121), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


