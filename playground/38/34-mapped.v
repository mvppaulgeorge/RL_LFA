// Benchmark "adder" written by ABC on Thu Jul 18 07:44:20 2024

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
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n276, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n340, new_n343, new_n345, new_n346, new_n347;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor022aa1n16x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  tech160nm_fiaoi012aa1n05x5   g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .out0(new_n103));
  xnrc02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .out0(new_n104));
  inv000aa1d42x5               g009(.a(\a[4] ), .o1(new_n105));
  inv000aa1d42x5               g010(.a(\b[3] ), .o1(new_n106));
  nor022aa1n08x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  oaoi03aa1n12x5               g012(.a(new_n105), .b(new_n106), .c(new_n107), .o1(new_n108));
  oai013aa1n06x5               g013(.a(new_n108), .b(new_n103), .c(new_n104), .d(new_n102), .o1(new_n109));
  nor042aa1n03x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n04x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norb02aa1n03x4               g016(.a(new_n111), .b(new_n110), .out0(new_n112));
  nand42aa1n06x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nor002aa1n16x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  norb02aa1n02x5               g019(.a(new_n113), .b(new_n114), .out0(new_n115));
  xnrc02aa1n12x5               g020(.a(\b[5] ), .b(\a[6] ), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .out0(new_n117));
  nano23aa1n06x5               g022(.a(new_n117), .b(new_n116), .c(new_n115), .d(new_n112), .out0(new_n118));
  nano23aa1n09x5               g023(.a(new_n114), .b(new_n110), .c(new_n111), .d(new_n113), .out0(new_n119));
  inv000aa1d42x5               g024(.a(new_n114), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[8] ), .b(\b[7] ), .c(new_n120), .o1(new_n121));
  orn002aa1n03x5               g026(.a(\a[5] ), .b(\b[4] ), .o(new_n122));
  oaoi03aa1n06x5               g027(.a(\a[6] ), .b(\b[5] ), .c(new_n122), .o1(new_n123));
  aoi012aa1n06x5               g028(.a(new_n121), .b(new_n119), .c(new_n123), .o1(new_n124));
  inv040aa1n03x5               g029(.a(new_n124), .o1(new_n125));
  nand02aa1n03x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n97), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n125), .c(new_n109), .d(new_n118), .o1(new_n128));
  nor002aa1n12x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  and002aa1n12x5               g034(.a(\b[9] ), .b(\a[10] ), .o(new_n130));
  nor042aa1n02x5               g035(.a(new_n130), .b(new_n129), .o1(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n128), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g037(.a(new_n129), .o1(new_n133));
  aoai13aa1n04x5               g038(.a(new_n133), .b(new_n130), .c(new_n128), .d(new_n98), .o1(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d32x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand42aa1n03x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  nor002aa1n06x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  tech160nm_finand02aa1n05x5   g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  aoi112aa1n03x4               g046(.a(new_n141), .b(new_n136), .c(new_n134), .d(new_n138), .o1(new_n142));
  inv000aa1d42x5               g047(.a(new_n136), .o1(new_n143));
  norp03aa1n02x5               g048(.a(new_n104), .b(new_n103), .c(new_n102), .o1(new_n144));
  inv000aa1d42x5               g049(.a(new_n108), .o1(new_n145));
  tech160nm_fioai012aa1n05x5   g050(.a(new_n118), .b(new_n144), .c(new_n145), .o1(new_n146));
  inv000aa1n02x5               g051(.a(new_n126), .o1(new_n147));
  aoai13aa1n02x5               g052(.a(new_n98), .b(new_n147), .c(new_n146), .d(new_n124), .o1(new_n148));
  inv000aa1n03x5               g053(.a(new_n130), .o1(new_n149));
  aoai13aa1n02x5               g054(.a(new_n138), .b(new_n129), .c(new_n148), .d(new_n149), .o1(new_n150));
  aobi12aa1n02x5               g055(.a(new_n141), .b(new_n150), .c(new_n143), .out0(new_n151));
  norp02aa1n02x5               g056(.a(new_n151), .b(new_n142), .o1(\s[12] ));
  nano23aa1n02x5               g057(.a(new_n139), .b(new_n136), .c(new_n140), .d(new_n137), .out0(new_n153));
  nona23aa1n03x5               g058(.a(new_n153), .b(new_n131), .c(new_n97), .d(new_n147), .out0(new_n154));
  nona23aa1n09x5               g059(.a(new_n137), .b(new_n140), .c(new_n139), .d(new_n136), .out0(new_n155));
  tech160nm_fioai012aa1n03p5x5 g060(.a(new_n140), .b(new_n139), .c(new_n136), .o1(new_n156));
  oai012aa1n06x5               g061(.a(new_n149), .b(new_n129), .c(new_n97), .o1(new_n157));
  oai012aa1n18x5               g062(.a(new_n156), .b(new_n155), .c(new_n157), .o1(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  aoai13aa1n03x5               g064(.a(new_n159), .b(new_n154), .c(new_n146), .d(new_n124), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g066(.a(\a[13] ), .o1(new_n162));
  inv000aa1d42x5               g067(.a(\b[12] ), .o1(new_n163));
  oaoi03aa1n02x5               g068(.a(new_n162), .b(new_n163), .c(new_n160), .o1(new_n164));
  xnrb03aa1n02x5               g069(.a(new_n164), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nano32aa1n02x4               g070(.a(new_n155), .b(new_n131), .c(new_n126), .d(new_n98), .out0(new_n166));
  aoai13aa1n02x5               g071(.a(new_n166), .b(new_n125), .c(new_n109), .d(new_n118), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  norp02aa1n04x5               g073(.a(\b[12] ), .b(\a[13] ), .o1(new_n169));
  nor022aa1n12x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nand42aa1n04x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nona23aa1n06x5               g076(.a(new_n168), .b(new_n171), .c(new_n170), .d(new_n169), .out0(new_n172));
  aoai13aa1n12x5               g077(.a(new_n171), .b(new_n170), .c(new_n162), .d(new_n163), .o1(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n172), .c(new_n167), .d(new_n159), .o1(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  xnrc02aa1n12x5               g081(.a(\b[14] ), .b(\a[15] ), .out0(new_n177));
  inv000aa1d42x5               g082(.a(new_n177), .o1(new_n178));
  xnrc02aa1n12x5               g083(.a(\b[15] ), .b(\a[16] ), .out0(new_n179));
  inv000aa1d42x5               g084(.a(new_n179), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(new_n176), .b(new_n180), .c(new_n174), .d(new_n178), .o1(new_n181));
  inv000aa1n02x5               g086(.a(new_n172), .o1(new_n182));
  inv000aa1d42x5               g087(.a(new_n173), .o1(new_n183));
  aoai13aa1n03x5               g088(.a(new_n178), .b(new_n183), .c(new_n160), .d(new_n182), .o1(new_n184));
  oaoi13aa1n02x7               g089(.a(new_n179), .b(new_n184), .c(\a[15] ), .d(\b[14] ), .o1(new_n185));
  norp02aa1n03x5               g090(.a(new_n185), .b(new_n181), .o1(\s[16] ));
  norp03aa1d12x5               g091(.a(new_n172), .b(new_n177), .c(new_n179), .o1(new_n187));
  nand02aa1n02x5               g092(.a(new_n166), .b(new_n187), .o1(new_n188));
  inv000aa1d42x5               g093(.a(new_n176), .o1(new_n189));
  oao003aa1n02x5               g094(.a(\a[16] ), .b(\b[15] ), .c(new_n189), .carry(new_n190));
  oai013aa1d12x5               g095(.a(new_n190), .b(new_n177), .c(new_n179), .d(new_n173), .o1(new_n191));
  aoi012aa1d24x5               g096(.a(new_n191), .b(new_n158), .c(new_n187), .o1(new_n192));
  aoai13aa1n12x5               g097(.a(new_n192), .b(new_n188), .c(new_n146), .d(new_n124), .o1(new_n193));
  xorb03aa1n02x5               g098(.a(new_n193), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d28x5               g099(.a(\a[17] ), .o1(new_n195));
  inv040aa1d32x5               g100(.a(\b[16] ), .o1(new_n196));
  tech160nm_fioaoi03aa1n03p5x5 g101(.a(new_n195), .b(new_n196), .c(new_n193), .o1(new_n197));
  nor022aa1n16x5               g102(.a(\b[17] ), .b(\a[18] ), .o1(new_n198));
  nand42aa1n20x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  nanb02aa1n06x5               g104(.a(new_n198), .b(new_n199), .out0(new_n200));
  tech160nm_fixorc02aa1n02p5x5 g105(.a(new_n197), .b(new_n200), .out0(\s[18] ));
  nor042aa1n02x5               g106(.a(new_n179), .b(new_n177), .o1(new_n202));
  nano22aa1n03x7               g107(.a(new_n154), .b(new_n182), .c(new_n202), .out0(new_n203));
  aoai13aa1n06x5               g108(.a(new_n203), .b(new_n125), .c(new_n109), .d(new_n118), .o1(new_n204));
  nanp02aa1n02x5               g109(.a(new_n196), .b(new_n195), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(\b[16] ), .b(\a[17] ), .o1(new_n206));
  nano22aa1n12x5               g111(.a(new_n200), .b(new_n205), .c(new_n206), .out0(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  aoai13aa1n12x5               g113(.a(new_n199), .b(new_n198), .c(new_n195), .d(new_n196), .o1(new_n209));
  aoai13aa1n02x7               g114(.a(new_n209), .b(new_n208), .c(new_n204), .d(new_n192), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n04x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nanp02aa1n04x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nanb02aa1n09x5               g119(.a(new_n213), .b(new_n214), .out0(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  inv000aa1d42x5               g121(.a(\b[19] ), .o1(new_n217));
  nanb02aa1d36x5               g122(.a(\a[20] ), .b(new_n217), .out0(new_n218));
  nand22aa1n04x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nanp02aa1n06x5               g124(.a(new_n218), .b(new_n219), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  aoi112aa1n03x4               g126(.a(new_n213), .b(new_n221), .c(new_n210), .d(new_n216), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n209), .o1(new_n223));
  aoai13aa1n06x5               g128(.a(new_n216), .b(new_n223), .c(new_n193), .d(new_n207), .o1(new_n224));
  oaoi13aa1n06x5               g129(.a(new_n220), .b(new_n224), .c(\a[19] ), .d(\b[18] ), .o1(new_n225));
  norp02aa1n03x5               g130(.a(new_n225), .b(new_n222), .o1(\s[20] ));
  nona22aa1n09x5               g131(.a(new_n207), .b(new_n215), .c(new_n220), .out0(new_n227));
  nanp02aa1n02x5               g132(.a(new_n213), .b(new_n219), .o1(new_n228));
  nor003aa1n03x5               g133(.a(new_n209), .b(new_n215), .c(new_n220), .o1(new_n229));
  nano22aa1n03x7               g134(.a(new_n229), .b(new_n218), .c(new_n228), .out0(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n227), .c(new_n204), .d(new_n192), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[20] ), .b(\a[21] ), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  xnrc02aa1n12x5               g140(.a(\b[21] ), .b(\a[22] ), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  aoi112aa1n02x5               g142(.a(new_n233), .b(new_n237), .c(new_n231), .d(new_n235), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n227), .o1(new_n239));
  nor042aa1n02x5               g144(.a(\b[19] ), .b(\a[20] ), .o1(new_n240));
  nona23aa1d18x5               g145(.a(new_n214), .b(new_n219), .c(new_n240), .d(new_n213), .out0(new_n241));
  oai112aa1n06x5               g146(.a(new_n228), .b(new_n218), .c(new_n241), .d(new_n209), .o1(new_n242));
  aoai13aa1n06x5               g147(.a(new_n235), .b(new_n242), .c(new_n193), .d(new_n239), .o1(new_n243));
  oaoi13aa1n06x5               g148(.a(new_n236), .b(new_n243), .c(\a[21] ), .d(\b[20] ), .o1(new_n244));
  norp02aa1n03x5               g149(.a(new_n244), .b(new_n238), .o1(\s[22] ));
  inv000aa1d42x5               g150(.a(new_n241), .o1(new_n246));
  nor042aa1n06x5               g151(.a(new_n236), .b(new_n234), .o1(new_n247));
  nand23aa1d12x5               g152(.a(new_n246), .b(new_n247), .c(new_n207), .o1(new_n248));
  inv000aa1d42x5               g153(.a(\a[22] ), .o1(new_n249));
  inv000aa1d42x5               g154(.a(\b[21] ), .o1(new_n250));
  oao003aa1n02x5               g155(.a(new_n249), .b(new_n250), .c(new_n233), .carry(new_n251));
  aoi012aa1n12x5               g156(.a(new_n251), .b(new_n242), .c(new_n247), .o1(new_n252));
  aoai13aa1n04x5               g157(.a(new_n252), .b(new_n248), .c(new_n204), .d(new_n192), .o1(new_n253));
  xorb03aa1n02x5               g158(.a(new_n253), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g159(.a(\b[22] ), .b(\a[23] ), .o1(new_n255));
  tech160nm_fixorc02aa1n03p5x5 g160(.a(\a[23] ), .b(\b[22] ), .out0(new_n256));
  xorc02aa1n12x5               g161(.a(\a[24] ), .b(\b[23] ), .out0(new_n257));
  aoi112aa1n03x4               g162(.a(new_n255), .b(new_n257), .c(new_n253), .d(new_n256), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n255), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n248), .o1(new_n260));
  inv000aa1n02x5               g165(.a(new_n252), .o1(new_n261));
  aoai13aa1n03x5               g166(.a(new_n256), .b(new_n261), .c(new_n193), .d(new_n260), .o1(new_n262));
  aobi12aa1n03x5               g167(.a(new_n257), .b(new_n262), .c(new_n259), .out0(new_n263));
  norp02aa1n03x5               g168(.a(new_n263), .b(new_n258), .o1(\s[24] ));
  nano32aa1n03x7               g169(.a(new_n227), .b(new_n257), .c(new_n247), .d(new_n256), .out0(new_n265));
  inv000aa1n02x5               g170(.a(new_n265), .o1(new_n266));
  xnrc02aa1n02x5               g171(.a(\b[22] ), .b(\a[23] ), .out0(new_n267));
  norb02aa1n03x5               g172(.a(new_n257), .b(new_n267), .out0(new_n268));
  norp02aa1n02x5               g173(.a(\b[23] ), .b(\a[24] ), .o1(new_n269));
  aoi112aa1n02x5               g174(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n270));
  nanp03aa1n02x5               g175(.a(new_n251), .b(new_n256), .c(new_n257), .o1(new_n271));
  nona22aa1n02x4               g176(.a(new_n271), .b(new_n270), .c(new_n269), .out0(new_n272));
  aoi013aa1n09x5               g177(.a(new_n272), .b(new_n242), .c(new_n247), .d(new_n268), .o1(new_n273));
  aoai13aa1n04x5               g178(.a(new_n273), .b(new_n266), .c(new_n204), .d(new_n192), .o1(new_n274));
  xorb03aa1n02x5               g179(.a(new_n274), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g180(.a(\b[24] ), .b(\a[25] ), .o1(new_n276));
  tech160nm_fixorc02aa1n02p5x5 g181(.a(\a[25] ), .b(\b[24] ), .out0(new_n277));
  xorc02aa1n02x5               g182(.a(\a[26] ), .b(\b[25] ), .out0(new_n278));
  aoi112aa1n03x4               g183(.a(new_n276), .b(new_n278), .c(new_n274), .d(new_n277), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n276), .o1(new_n280));
  inv000aa1n02x5               g185(.a(new_n273), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n277), .b(new_n281), .c(new_n193), .d(new_n265), .o1(new_n282));
  aobi12aa1n03x5               g187(.a(new_n278), .b(new_n282), .c(new_n280), .out0(new_n283));
  nor002aa1n02x5               g188(.a(new_n283), .b(new_n279), .o1(\s[26] ));
  xorc02aa1n02x5               g189(.a(\a[4] ), .b(\b[3] ), .out0(new_n285));
  xorc02aa1n02x5               g190(.a(\a[3] ), .b(\b[2] ), .out0(new_n286));
  nanb03aa1n02x5               g191(.a(new_n102), .b(new_n286), .c(new_n285), .out0(new_n287));
  nona22aa1n02x4               g192(.a(new_n119), .b(new_n116), .c(new_n117), .out0(new_n288));
  aoai13aa1n06x5               g193(.a(new_n124), .b(new_n288), .c(new_n287), .d(new_n108), .o1(new_n289));
  inv040aa1n03x5               g194(.a(new_n191), .o1(new_n290));
  aob012aa1n03x5               g195(.a(new_n290), .b(new_n158), .c(new_n187), .out0(new_n291));
  and002aa1n06x5               g196(.a(new_n278), .b(new_n277), .o(new_n292));
  nano22aa1d15x5               g197(.a(new_n248), .b(new_n292), .c(new_n268), .out0(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n291), .c(new_n289), .d(new_n203), .o1(new_n294));
  nano22aa1n03x7               g199(.a(new_n230), .b(new_n247), .c(new_n268), .out0(new_n295));
  oao003aa1n02x5               g200(.a(\a[26] ), .b(\b[25] ), .c(new_n280), .carry(new_n296));
  inv000aa1d42x5               g201(.a(new_n296), .o1(new_n297));
  oaoi13aa1n09x5               g202(.a(new_n297), .b(new_n292), .c(new_n295), .d(new_n272), .o1(new_n298));
  xorc02aa1n02x5               g203(.a(\a[27] ), .b(\b[26] ), .out0(new_n299));
  xnbna2aa1n03x5               g204(.a(new_n299), .b(new_n298), .c(new_n294), .out0(\s[27] ));
  nor042aa1n03x5               g205(.a(\b[26] ), .b(\a[27] ), .o1(new_n301));
  inv000aa1d42x5               g206(.a(new_n301), .o1(new_n302));
  aobi12aa1n06x5               g207(.a(new_n299), .b(new_n298), .c(new_n294), .out0(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[27] ), .b(\a[28] ), .out0(new_n304));
  nano22aa1n03x7               g209(.a(new_n303), .b(new_n302), .c(new_n304), .out0(new_n305));
  nanp02aa1n02x5               g210(.a(new_n257), .b(new_n256), .o1(new_n306));
  nona32aa1n06x5               g211(.a(new_n242), .b(new_n306), .c(new_n236), .d(new_n234), .out0(new_n307));
  aoi113aa1n02x5               g212(.a(new_n270), .b(new_n269), .c(new_n251), .d(new_n257), .e(new_n256), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n292), .o1(new_n309));
  aoai13aa1n06x5               g214(.a(new_n296), .b(new_n309), .c(new_n307), .d(new_n308), .o1(new_n310));
  aoai13aa1n03x5               g215(.a(new_n299), .b(new_n310), .c(new_n193), .d(new_n293), .o1(new_n311));
  aoi012aa1n03x5               g216(.a(new_n304), .b(new_n311), .c(new_n302), .o1(new_n312));
  nor002aa1n02x5               g217(.a(new_n312), .b(new_n305), .o1(\s[28] ));
  xnrc02aa1n02x5               g218(.a(\b[28] ), .b(\a[29] ), .out0(new_n314));
  norb02aa1n02x5               g219(.a(new_n299), .b(new_n304), .out0(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n310), .c(new_n193), .d(new_n293), .o1(new_n316));
  oao003aa1n02x5               g221(.a(\a[28] ), .b(\b[27] ), .c(new_n302), .carry(new_n317));
  aoi012aa1n03x5               g222(.a(new_n314), .b(new_n316), .c(new_n317), .o1(new_n318));
  aobi12aa1n06x5               g223(.a(new_n315), .b(new_n298), .c(new_n294), .out0(new_n319));
  nano22aa1n03x7               g224(.a(new_n319), .b(new_n314), .c(new_n317), .out0(new_n320));
  nor002aa1n02x5               g225(.a(new_n318), .b(new_n320), .o1(\s[29] ));
  xorb03aa1n02x5               g226(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g227(.a(new_n299), .b(new_n314), .c(new_n304), .out0(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n310), .c(new_n193), .d(new_n293), .o1(new_n324));
  oao003aa1n02x5               g229(.a(\a[29] ), .b(\b[28] ), .c(new_n317), .carry(new_n325));
  xnrc02aa1n02x5               g230(.a(\b[29] ), .b(\a[30] ), .out0(new_n326));
  aoi012aa1n03x5               g231(.a(new_n326), .b(new_n324), .c(new_n325), .o1(new_n327));
  aobi12aa1n02x7               g232(.a(new_n323), .b(new_n298), .c(new_n294), .out0(new_n328));
  nano22aa1n02x4               g233(.a(new_n328), .b(new_n325), .c(new_n326), .out0(new_n329));
  nor002aa1n02x5               g234(.a(new_n327), .b(new_n329), .o1(\s[30] ));
  xnrc02aa1n02x5               g235(.a(\b[30] ), .b(\a[31] ), .out0(new_n331));
  norb02aa1n02x5               g236(.a(new_n323), .b(new_n326), .out0(new_n332));
  aobi12aa1n02x7               g237(.a(new_n332), .b(new_n298), .c(new_n294), .out0(new_n333));
  oao003aa1n02x5               g238(.a(\a[30] ), .b(\b[29] ), .c(new_n325), .carry(new_n334));
  nano22aa1n03x5               g239(.a(new_n333), .b(new_n331), .c(new_n334), .out0(new_n335));
  aoai13aa1n03x5               g240(.a(new_n332), .b(new_n310), .c(new_n193), .d(new_n293), .o1(new_n336));
  aoi012aa1n03x5               g241(.a(new_n331), .b(new_n336), .c(new_n334), .o1(new_n337));
  norp02aa1n03x5               g242(.a(new_n337), .b(new_n335), .o1(\s[31] ));
  xnrb03aa1n02x5               g243(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g244(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n340));
  xorb03aa1n02x5               g245(.a(new_n340), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g246(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoai13aa1n02x5               g247(.a(new_n122), .b(new_n117), .c(new_n287), .d(new_n108), .o1(new_n343));
  xorb03aa1n02x5               g248(.a(new_n343), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g249(.a(new_n116), .o1(new_n345));
  aoai13aa1n03x5               g250(.a(new_n115), .b(new_n123), .c(new_n343), .d(new_n345), .o1(new_n346));
  aoi112aa1n02x5               g251(.a(new_n123), .b(new_n115), .c(new_n343), .d(new_n345), .o1(new_n347));
  norb02aa1n02x5               g252(.a(new_n346), .b(new_n347), .out0(\s[7] ));
  xnbna2aa1n03x5               g253(.a(new_n112), .b(new_n346), .c(new_n120), .out0(\s[8] ));
  xorb03aa1n02x5               g254(.a(new_n289), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


