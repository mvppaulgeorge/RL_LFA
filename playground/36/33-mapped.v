// Benchmark "adder" written by ABC on Thu Jul 18 06:42:11 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n341, new_n344, new_n346, new_n348;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n12x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nanp02aa1n03x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nand22aa1n03x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  aoi012aa1n12x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nor042aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanb02aa1n06x5               g007(.a(new_n102), .b(new_n101), .out0(new_n103));
  inv000aa1d42x5               g008(.a(\a[3] ), .o1(new_n104));
  inv000aa1d42x5               g009(.a(\b[2] ), .o1(new_n105));
  nand02aa1d12x5               g010(.a(new_n105), .b(new_n104), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nanp02aa1n04x5               g012(.a(new_n106), .b(new_n107), .o1(new_n108));
  oaoi03aa1n06x5               g013(.a(\a[4] ), .b(\b[3] ), .c(new_n106), .o1(new_n109));
  inv000aa1n02x5               g014(.a(new_n109), .o1(new_n110));
  oai013aa1n09x5               g015(.a(new_n110), .b(new_n100), .c(new_n103), .d(new_n108), .o1(new_n111));
  nand22aa1n03x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nor002aa1n06x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nor042aa1n04x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nona23aa1n02x4               g020(.a(new_n112), .b(new_n115), .c(new_n114), .d(new_n113), .out0(new_n116));
  nor042aa1n02x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nor002aa1n03x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  nona23aa1n03x5               g025(.a(new_n119), .b(new_n118), .c(new_n120), .d(new_n117), .out0(new_n121));
  nor042aa1n03x5               g026(.a(new_n121), .b(new_n116), .o1(new_n122));
  tech160nm_fiaoi012aa1n05x5   g027(.a(new_n113), .b(new_n114), .c(new_n112), .o1(new_n123));
  tech160nm_fiao0012aa1n02p5x5 g028(.a(new_n117), .b(new_n120), .c(new_n118), .o(new_n124));
  oabi12aa1n03x5               g029(.a(new_n124), .b(new_n121), .c(new_n123), .out0(new_n125));
  nor022aa1n16x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n125), .c(new_n111), .d(new_n122), .o1(new_n129));
  oai012aa1n02x5               g034(.a(new_n129), .b(\b[8] ), .c(\a[9] ), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand02aa1n06x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nor022aa1n08x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nanp02aa1n04x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n06x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  norp02aa1n12x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  nona22aa1n03x5               g041(.a(new_n129), .b(new_n136), .c(new_n126), .out0(new_n137));
  xobna2aa1n03x5               g042(.a(new_n135), .b(new_n137), .c(new_n132), .out0(\s[11] ));
  aoi013aa1n02x4               g043(.a(new_n133), .b(new_n137), .c(new_n135), .d(new_n132), .o1(new_n139));
  norp02aa1n04x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n03x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  norb02aa1n03x5               g047(.a(new_n142), .b(new_n139), .out0(new_n143));
  aoi113aa1n02x5               g048(.a(new_n133), .b(new_n142), .c(new_n137), .d(new_n134), .e(new_n132), .o1(new_n144));
  nor002aa1n02x5               g049(.a(new_n143), .b(new_n144), .o1(\s[12] ));
  nor003aa1n03x5               g050(.a(new_n100), .b(new_n103), .c(new_n108), .o1(new_n146));
  tech160nm_fioai012aa1n05x5   g051(.a(new_n122), .b(new_n146), .c(new_n109), .o1(new_n147));
  nano23aa1n06x5               g052(.a(new_n120), .b(new_n117), .c(new_n118), .d(new_n119), .out0(new_n148));
  inv000aa1n06x5               g053(.a(new_n123), .o1(new_n149));
  aoi012aa1n12x5               g054(.a(new_n124), .b(new_n148), .c(new_n149), .o1(new_n150));
  nona23aa1n03x5               g055(.a(new_n132), .b(new_n127), .c(new_n126), .d(new_n136), .out0(new_n151));
  nano22aa1n06x5               g056(.a(new_n151), .b(new_n135), .c(new_n142), .out0(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  oai112aa1n06x5               g058(.a(new_n132), .b(new_n134), .c(new_n136), .d(new_n126), .o1(new_n154));
  nor002aa1n02x5               g059(.a(new_n140), .b(new_n133), .o1(new_n155));
  aob012aa1n02x5               g060(.a(new_n141), .b(new_n154), .c(new_n155), .out0(new_n156));
  aoai13aa1n03x5               g061(.a(new_n156), .b(new_n153), .c(new_n147), .d(new_n150), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n06x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  aoi012aa1n02x5               g065(.a(new_n159), .b(new_n157), .c(new_n160), .o1(new_n161));
  xnrb03aa1n02x5               g066(.a(new_n161), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  aoai13aa1n02x5               g067(.a(new_n152), .b(new_n125), .c(new_n111), .d(new_n122), .o1(new_n163));
  nor002aa1n12x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nand22aa1n09x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nona23aa1n03x5               g070(.a(new_n165), .b(new_n160), .c(new_n159), .d(new_n164), .out0(new_n166));
  aoi012aa1d24x5               g071(.a(new_n164), .b(new_n159), .c(new_n165), .o1(new_n167));
  aoai13aa1n03x5               g072(.a(new_n167), .b(new_n166), .c(new_n163), .d(new_n156), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n03x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  inv000aa1n02x5               g075(.a(new_n170), .o1(new_n171));
  nano23aa1n03x5               g076(.a(new_n159), .b(new_n164), .c(new_n165), .d(new_n160), .out0(new_n172));
  inv000aa1d42x5               g077(.a(new_n167), .o1(new_n173));
  nand42aa1n02x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nanb02aa1n02x5               g079(.a(new_n170), .b(new_n174), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n173), .c(new_n157), .d(new_n172), .o1(new_n177));
  nor042aa1n06x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nand42aa1d28x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  norb02aa1n12x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  inv000aa1d42x5               g085(.a(new_n180), .o1(new_n181));
  tech160nm_fiaoi012aa1n02p5x5 g086(.a(new_n181), .b(new_n177), .c(new_n171), .o1(new_n182));
  aoi112aa1n02x5               g087(.a(new_n170), .b(new_n180), .c(new_n168), .d(new_n174), .o1(new_n183));
  norp02aa1n02x5               g088(.a(new_n182), .b(new_n183), .o1(\s[16] ));
  nano23aa1n02x4               g089(.a(new_n126), .b(new_n136), .c(new_n132), .d(new_n127), .out0(new_n185));
  nano23aa1n02x4               g090(.a(new_n170), .b(new_n178), .c(new_n179), .d(new_n174), .out0(new_n186));
  nand02aa1n02x5               g091(.a(new_n186), .b(new_n172), .o1(new_n187));
  nano32aa1n03x7               g092(.a(new_n187), .b(new_n185), .c(new_n142), .d(new_n135), .out0(new_n188));
  aoai13aa1n06x5               g093(.a(new_n188), .b(new_n125), .c(new_n111), .d(new_n122), .o1(new_n189));
  aoi022aa1n03x5               g094(.a(new_n154), .b(new_n155), .c(\b[11] ), .d(\a[12] ), .o1(new_n190));
  nano32aa1n03x7               g095(.a(new_n166), .b(new_n180), .c(new_n171), .d(new_n174), .out0(new_n191));
  aoai13aa1n02x7               g096(.a(new_n174), .b(new_n164), .c(new_n159), .d(new_n165), .o1(new_n192));
  nona22aa1n03x5               g097(.a(new_n192), .b(new_n178), .c(new_n170), .out0(new_n193));
  aoi022aa1n12x5               g098(.a(new_n191), .b(new_n190), .c(new_n179), .d(new_n193), .o1(new_n194));
  nor042aa1d18x5               g099(.a(\b[16] ), .b(\a[17] ), .o1(new_n195));
  nand42aa1d28x5               g100(.a(\b[16] ), .b(\a[17] ), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  xnbna2aa1n03x5               g102(.a(new_n197), .b(new_n189), .c(new_n194), .out0(\s[17] ));
  nand22aa1n03x5               g103(.a(new_n191), .b(new_n152), .o1(new_n199));
  aoai13aa1n12x5               g104(.a(new_n194), .b(new_n199), .c(new_n147), .d(new_n150), .o1(new_n200));
  tech160nm_fiaoi012aa1n05x5   g105(.a(new_n195), .b(new_n200), .c(new_n197), .o1(new_n201));
  xnrb03aa1n03x5               g106(.a(new_n201), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor042aa1n06x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  nand42aa1d28x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  nano23aa1d15x5               g109(.a(new_n195), .b(new_n203), .c(new_n204), .d(new_n196), .out0(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  aoi012aa1n02x5               g111(.a(new_n203), .b(new_n195), .c(new_n204), .o1(new_n207));
  aoai13aa1n06x5               g112(.a(new_n207), .b(new_n206), .c(new_n189), .d(new_n194), .o1(new_n208));
  xorb03aa1n02x5               g113(.a(new_n208), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g114(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d24x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nanp02aa1n06x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  xnrc02aa1n12x5               g117(.a(\b[19] ), .b(\a[20] ), .out0(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  aoi112aa1n02x5               g119(.a(new_n211), .b(new_n214), .c(new_n208), .d(new_n212), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n211), .o1(new_n216));
  inv000aa1n02x5               g121(.a(new_n207), .o1(new_n217));
  nanb02aa1n18x5               g122(.a(new_n211), .b(new_n212), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  aoai13aa1n03x5               g124(.a(new_n219), .b(new_n217), .c(new_n200), .d(new_n205), .o1(new_n220));
  aoi012aa1n03x5               g125(.a(new_n213), .b(new_n220), .c(new_n216), .o1(new_n221));
  norp02aa1n03x5               g126(.a(new_n221), .b(new_n215), .o1(\s[20] ));
  nona22aa1d24x5               g127(.a(new_n205), .b(new_n213), .c(new_n218), .out0(new_n223));
  aoai13aa1n06x5               g128(.a(new_n212), .b(new_n203), .c(new_n195), .d(new_n204), .o1(new_n224));
  oab012aa1n06x5               g129(.a(new_n211), .b(\a[20] ), .c(\b[19] ), .out0(new_n225));
  aoi022aa1n12x5               g130(.a(new_n224), .b(new_n225), .c(\b[19] ), .d(\a[20] ), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  aoai13aa1n04x5               g132(.a(new_n227), .b(new_n223), .c(new_n189), .d(new_n194), .o1(new_n228));
  xorb03aa1n02x5               g133(.a(new_n228), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1d32x5               g134(.a(\b[20] ), .b(\a[21] ), .o1(new_n230));
  nand42aa1n02x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  norb02aa1n02x5               g136(.a(new_n231), .b(new_n230), .out0(new_n232));
  nor042aa1d18x5               g137(.a(\b[21] ), .b(\a[22] ), .o1(new_n233));
  nand42aa1n08x5               g138(.a(\b[21] ), .b(\a[22] ), .o1(new_n234));
  nanb02aa1n12x5               g139(.a(new_n233), .b(new_n234), .out0(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  aoi112aa1n02x5               g141(.a(new_n230), .b(new_n236), .c(new_n228), .d(new_n232), .o1(new_n237));
  inv000aa1n02x5               g142(.a(new_n230), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n223), .o1(new_n239));
  aoai13aa1n03x5               g144(.a(new_n232), .b(new_n226), .c(new_n200), .d(new_n239), .o1(new_n240));
  tech160nm_fiaoi012aa1n02p5x5 g145(.a(new_n235), .b(new_n240), .c(new_n238), .o1(new_n241));
  nor002aa1n02x5               g146(.a(new_n241), .b(new_n237), .o1(\s[22] ));
  nano23aa1n06x5               g147(.a(new_n230), .b(new_n233), .c(new_n234), .d(new_n231), .out0(new_n243));
  nona23aa1d16x5               g148(.a(new_n205), .b(new_n243), .c(new_n213), .d(new_n218), .out0(new_n244));
  nand02aa1d06x5               g149(.a(new_n224), .b(new_n225), .o1(new_n245));
  nanp02aa1n02x5               g150(.a(\b[19] ), .b(\a[20] ), .o1(new_n246));
  nano32aa1n03x7               g151(.a(new_n235), .b(new_n238), .c(new_n231), .d(new_n246), .out0(new_n247));
  oaoi03aa1n02x5               g152(.a(\a[22] ), .b(\b[21] ), .c(new_n238), .o1(new_n248));
  aoi012aa1n02x5               g153(.a(new_n248), .b(new_n245), .c(new_n247), .o1(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n244), .c(new_n189), .d(new_n194), .o1(new_n250));
  xorb03aa1n02x5               g155(.a(new_n250), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n06x5               g156(.a(\b[22] ), .b(\a[23] ), .o1(new_n252));
  tech160nm_fixorc02aa1n03p5x5 g157(.a(\a[23] ), .b(\b[22] ), .out0(new_n253));
  xorc02aa1n12x5               g158(.a(\a[24] ), .b(\b[23] ), .out0(new_n254));
  aoi112aa1n02x5               g159(.a(new_n252), .b(new_n254), .c(new_n250), .d(new_n253), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n252), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n244), .o1(new_n257));
  inv000aa1n02x5               g162(.a(new_n249), .o1(new_n258));
  aoai13aa1n03x5               g163(.a(new_n253), .b(new_n258), .c(new_n200), .d(new_n257), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n254), .o1(new_n260));
  aoi012aa1n02x7               g165(.a(new_n260), .b(new_n259), .c(new_n256), .o1(new_n261));
  nor002aa1n02x5               g166(.a(new_n261), .b(new_n255), .o1(\s[24] ));
  nano32aa1n03x7               g167(.a(new_n223), .b(new_n254), .c(new_n243), .d(new_n253), .out0(new_n263));
  inv030aa1n02x5               g168(.a(new_n263), .o1(new_n264));
  and002aa1n06x5               g169(.a(new_n254), .b(new_n253), .o(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n248), .c(new_n245), .d(new_n247), .o1(new_n266));
  oao003aa1n02x5               g171(.a(\a[24] ), .b(\b[23] ), .c(new_n256), .carry(new_n267));
  nand22aa1n12x5               g172(.a(new_n266), .b(new_n267), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n268), .o1(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n264), .c(new_n189), .d(new_n194), .o1(new_n270));
  xorb03aa1n02x5               g175(.a(new_n270), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g176(.a(\b[24] ), .b(\a[25] ), .o1(new_n272));
  xorc02aa1n02x5               g177(.a(\a[25] ), .b(\b[24] ), .out0(new_n273));
  xorc02aa1n12x5               g178(.a(\a[26] ), .b(\b[25] ), .out0(new_n274));
  aoi112aa1n02x5               g179(.a(new_n272), .b(new_n274), .c(new_n270), .d(new_n273), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n272), .o1(new_n276));
  aoai13aa1n03x5               g181(.a(new_n273), .b(new_n268), .c(new_n200), .d(new_n263), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n274), .o1(new_n278));
  tech160nm_fiaoi012aa1n02p5x5 g183(.a(new_n278), .b(new_n277), .c(new_n276), .o1(new_n279));
  nor042aa1n03x5               g184(.a(new_n279), .b(new_n275), .o1(\s[26] ));
  inv000aa1d42x5               g185(.a(new_n97), .o1(new_n281));
  aob012aa1n02x5               g186(.a(new_n281), .b(new_n98), .c(new_n99), .out0(new_n282));
  norb02aa1n02x5               g187(.a(new_n101), .b(new_n102), .out0(new_n283));
  norp02aa1n02x5               g188(.a(\b[2] ), .b(\a[3] ), .o1(new_n284));
  norb02aa1n02x5               g189(.a(new_n107), .b(new_n284), .out0(new_n285));
  nanp03aa1n02x5               g190(.a(new_n282), .b(new_n283), .c(new_n285), .o1(new_n286));
  nano23aa1n02x4               g191(.a(new_n114), .b(new_n113), .c(new_n115), .d(new_n112), .out0(new_n287));
  nanp02aa1n02x5               g192(.a(new_n148), .b(new_n287), .o1(new_n288));
  aoai13aa1n02x7               g193(.a(new_n150), .b(new_n288), .c(new_n286), .d(new_n110), .o1(new_n289));
  nanp02aa1n02x5               g194(.a(new_n193), .b(new_n179), .o1(new_n290));
  oai012aa1n02x5               g195(.a(new_n290), .b(new_n156), .c(new_n187), .o1(new_n291));
  and002aa1n02x5               g196(.a(new_n274), .b(new_n273), .o(new_n292));
  nano22aa1d15x5               g197(.a(new_n244), .b(new_n292), .c(new_n265), .out0(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n291), .c(new_n289), .d(new_n188), .o1(new_n294));
  nanp02aa1n02x5               g199(.a(\b[25] ), .b(\a[26] ), .o1(new_n295));
  oai022aa1n02x5               g200(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n296));
  aoi022aa1n12x5               g201(.a(new_n268), .b(new_n292), .c(new_n295), .d(new_n296), .o1(new_n297));
  xorc02aa1n12x5               g202(.a(\a[27] ), .b(\b[26] ), .out0(new_n298));
  xnbna2aa1n03x5               g203(.a(new_n298), .b(new_n297), .c(new_n294), .out0(\s[27] ));
  norp02aa1n02x5               g204(.a(\b[26] ), .b(\a[27] ), .o1(new_n300));
  inv040aa1n03x5               g205(.a(new_n300), .o1(new_n301));
  inv000aa1d42x5               g206(.a(new_n298), .o1(new_n302));
  tech160nm_fiaoi012aa1n02p5x5 g207(.a(new_n302), .b(new_n297), .c(new_n294), .o1(new_n303));
  xnrc02aa1n12x5               g208(.a(\b[27] ), .b(\a[28] ), .out0(new_n304));
  nano22aa1n03x5               g209(.a(new_n303), .b(new_n301), .c(new_n304), .out0(new_n305));
  inv000aa1n02x5               g210(.a(new_n292), .o1(new_n306));
  nanp02aa1n02x5               g211(.a(new_n296), .b(new_n295), .o1(new_n307));
  aoai13aa1n04x5               g212(.a(new_n307), .b(new_n306), .c(new_n266), .d(new_n267), .o1(new_n308));
  aoai13aa1n03x5               g213(.a(new_n298), .b(new_n308), .c(new_n200), .d(new_n293), .o1(new_n309));
  tech160nm_fiaoi012aa1n02p5x5 g214(.a(new_n304), .b(new_n309), .c(new_n301), .o1(new_n310));
  norp02aa1n03x5               g215(.a(new_n310), .b(new_n305), .o1(\s[28] ));
  xnrc02aa1n02x5               g216(.a(\b[28] ), .b(\a[29] ), .out0(new_n312));
  norb02aa1n06x5               g217(.a(new_n298), .b(new_n304), .out0(new_n313));
  aoai13aa1n03x5               g218(.a(new_n313), .b(new_n308), .c(new_n200), .d(new_n293), .o1(new_n314));
  oao003aa1n02x5               g219(.a(\a[28] ), .b(\b[27] ), .c(new_n301), .carry(new_n315));
  tech160nm_fiaoi012aa1n02p5x5 g220(.a(new_n312), .b(new_n314), .c(new_n315), .o1(new_n316));
  inv000aa1d42x5               g221(.a(new_n313), .o1(new_n317));
  tech160nm_fiaoi012aa1n02p5x5 g222(.a(new_n317), .b(new_n297), .c(new_n294), .o1(new_n318));
  nano22aa1n03x5               g223(.a(new_n318), .b(new_n312), .c(new_n315), .out0(new_n319));
  norp02aa1n03x5               g224(.a(new_n316), .b(new_n319), .o1(\s[29] ));
  xorb03aa1n02x5               g225(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n06x5               g226(.a(new_n298), .b(new_n312), .c(new_n304), .out0(new_n322));
  aoai13aa1n03x5               g227(.a(new_n322), .b(new_n308), .c(new_n200), .d(new_n293), .o1(new_n323));
  oao003aa1n02x5               g228(.a(\a[29] ), .b(\b[28] ), .c(new_n315), .carry(new_n324));
  xnrc02aa1n02x5               g229(.a(\b[29] ), .b(\a[30] ), .out0(new_n325));
  tech160nm_fiaoi012aa1n02p5x5 g230(.a(new_n325), .b(new_n323), .c(new_n324), .o1(new_n326));
  inv000aa1d42x5               g231(.a(new_n322), .o1(new_n327));
  tech160nm_fiaoi012aa1n02p5x5 g232(.a(new_n327), .b(new_n297), .c(new_n294), .o1(new_n328));
  nano22aa1n03x5               g233(.a(new_n328), .b(new_n324), .c(new_n325), .out0(new_n329));
  norp02aa1n03x5               g234(.a(new_n326), .b(new_n329), .o1(\s[30] ));
  nona32aa1n02x4               g235(.a(new_n298), .b(new_n325), .c(new_n312), .d(new_n304), .out0(new_n331));
  tech160nm_fiaoi012aa1n02p5x5 g236(.a(new_n331), .b(new_n297), .c(new_n294), .o1(new_n332));
  oao003aa1n02x5               g237(.a(\a[30] ), .b(\b[29] ), .c(new_n324), .carry(new_n333));
  xnrc02aa1n02x5               g238(.a(\b[30] ), .b(\a[31] ), .out0(new_n334));
  nano22aa1n03x5               g239(.a(new_n332), .b(new_n333), .c(new_n334), .out0(new_n335));
  inv000aa1n02x5               g240(.a(new_n331), .o1(new_n336));
  aoai13aa1n03x5               g241(.a(new_n336), .b(new_n308), .c(new_n200), .d(new_n293), .o1(new_n337));
  tech160nm_fiaoi012aa1n02p5x5 g242(.a(new_n334), .b(new_n337), .c(new_n333), .o1(new_n338));
  norp02aa1n03x5               g243(.a(new_n338), .b(new_n335), .o1(\s[31] ));
  xnbna2aa1n03x5               g244(.a(new_n100), .b(new_n107), .c(new_n106), .out0(\s[3] ));
  oaoi03aa1n02x5               g245(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n341));
  xorb03aa1n02x5               g246(.a(new_n341), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g247(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g248(.a(new_n114), .b(new_n111), .c(new_n115), .o1(new_n344));
  xnrb03aa1n02x5               g249(.a(new_n344), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g250(.a(new_n123), .b(new_n116), .c(new_n286), .d(new_n110), .o1(new_n346));
  xorb03aa1n02x5               g251(.a(new_n346), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g252(.a(new_n120), .b(new_n346), .c(new_n119), .o1(new_n348));
  xnrb03aa1n02x5               g253(.a(new_n348), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g254(.a(new_n289), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


