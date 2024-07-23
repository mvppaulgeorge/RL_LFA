// Benchmark "adder" written by ABC on Thu Jul 18 07:10:31 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n255, new_n256, new_n257, new_n258, new_n259, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n314, new_n317, new_n318, new_n320,
    new_n322;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor042aa1d18x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand02aa1d04x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nor042aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  oai012aa1n12x5               g009(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n105));
  nand42aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nor022aa1n16x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nor042aa1n06x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  nand02aa1n04x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  nona23aa1n12x5               g014(.a(new_n106), .b(new_n109), .c(new_n108), .d(new_n107), .out0(new_n110));
  oa0012aa1n06x5               g015(.a(new_n109), .b(new_n108), .c(new_n107), .o(new_n111));
  oabi12aa1n18x5               g016(.a(new_n111), .b(new_n105), .c(new_n110), .out0(new_n112));
  nor002aa1d32x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand02aa1d24x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nor042aa1n06x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nanp02aa1n04x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nona23aa1n09x5               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  xorc02aa1n02x5               g022(.a(\a[8] ), .b(\b[7] ), .out0(new_n118));
  tech160nm_fixorc02aa1n05x5   g023(.a(\a[6] ), .b(\b[5] ), .out0(new_n119));
  nano22aa1n06x5               g024(.a(new_n117), .b(new_n118), .c(new_n119), .out0(new_n120));
  inv000aa1d42x5               g025(.a(new_n113), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\a[8] ), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\b[7] ), .o1(new_n123));
  nanp02aa1n02x5               g028(.a(new_n123), .b(new_n122), .o1(new_n124));
  and002aa1n02x5               g029(.a(\b[7] ), .b(\a[8] ), .o(new_n125));
  norb02aa1n12x5               g030(.a(new_n114), .b(new_n113), .out0(new_n126));
  nor042aa1n03x5               g031(.a(\b[5] ), .b(\a[6] ), .o1(new_n127));
  aoi112aa1n09x5               g032(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n128));
  oaih12aa1n12x5               g033(.a(new_n126), .b(new_n128), .c(new_n127), .o1(new_n129));
  aoai13aa1n12x5               g034(.a(new_n124), .b(new_n125), .c(new_n129), .d(new_n121), .o1(new_n130));
  xorc02aa1n12x5               g035(.a(\a[9] ), .b(\b[8] ), .out0(new_n131));
  aoai13aa1n03x5               g036(.a(new_n131), .b(new_n130), .c(new_n112), .d(new_n120), .o1(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n99), .b(new_n132), .c(new_n101), .out0(\s[10] ));
  inv000aa1d42x5               g038(.a(new_n97), .o1(new_n134));
  inv000aa1n03x5               g039(.a(new_n98), .o1(new_n135));
  aoai13aa1n03x5               g040(.a(new_n134), .b(new_n135), .c(new_n132), .d(new_n101), .o1(new_n136));
  xorb03aa1n02x5               g041(.a(new_n136), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d32x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nand42aa1n16x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nor002aa1d32x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand02aa1d16x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  aoai13aa1n03x5               g047(.a(new_n142), .b(new_n138), .c(new_n136), .d(new_n139), .o1(new_n143));
  aoi112aa1n02x5               g048(.a(new_n138), .b(new_n142), .c(new_n136), .d(new_n139), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n143), .b(new_n144), .out0(\s[12] ));
  nona32aa1n09x5               g050(.a(new_n131), .b(new_n135), .c(new_n138), .d(new_n97), .out0(new_n146));
  nanb03aa1d24x5               g051(.a(new_n140), .b(new_n141), .c(new_n139), .out0(new_n147));
  norp02aa1n02x5               g052(.a(new_n146), .b(new_n147), .o1(new_n148));
  aoai13aa1n06x5               g053(.a(new_n148), .b(new_n130), .c(new_n112), .d(new_n120), .o1(new_n149));
  aoi012aa1n12x5               g054(.a(new_n97), .b(new_n100), .c(new_n98), .o1(new_n150));
  tech160nm_fiaoi012aa1n05x5   g055(.a(new_n140), .b(new_n138), .c(new_n141), .o1(new_n151));
  oai013aa1d12x5               g056(.a(new_n151), .b(new_n147), .c(new_n150), .d(new_n138), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  nor002aa1d24x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nand42aa1d28x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  norb02aa1n15x5               g060(.a(new_n155), .b(new_n154), .out0(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n156), .b(new_n149), .c(new_n153), .out0(\s[13] ));
  nanp02aa1n02x5               g062(.a(new_n149), .b(new_n153), .o1(new_n158));
  aoi012aa1n02x5               g063(.a(new_n154), .b(new_n158), .c(new_n155), .o1(new_n159));
  xnrb03aa1n02x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n06x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand22aa1n12x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nona23aa1n03x5               g067(.a(new_n162), .b(new_n155), .c(new_n154), .d(new_n161), .out0(new_n163));
  aoi012aa1n02x7               g068(.a(new_n161), .b(new_n154), .c(new_n162), .o1(new_n164));
  aoai13aa1n03x5               g069(.a(new_n164), .b(new_n163), .c(new_n149), .d(new_n153), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n16x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nand02aa1n04x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nor002aa1d32x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nand02aa1d08x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nanb02aa1n12x5               g075(.a(new_n169), .b(new_n170), .out0(new_n171));
  inv000aa1d42x5               g076(.a(new_n171), .o1(new_n172));
  aoai13aa1n02x7               g077(.a(new_n172), .b(new_n167), .c(new_n165), .d(new_n168), .o1(new_n173));
  aoi112aa1n02x5               g078(.a(new_n167), .b(new_n172), .c(new_n165), .d(new_n168), .o1(new_n174));
  norb02aa1n03x4               g079(.a(new_n173), .b(new_n174), .out0(\s[16] ));
  nano22aa1n03x7               g080(.a(new_n140), .b(new_n139), .c(new_n141), .out0(new_n176));
  nona23aa1d18x5               g081(.a(new_n156), .b(new_n162), .c(new_n167), .d(new_n161), .out0(new_n177));
  nano22aa1d15x5               g082(.a(new_n169), .b(new_n168), .c(new_n170), .out0(new_n178));
  nano23aa1d15x5               g083(.a(new_n146), .b(new_n177), .c(new_n178), .d(new_n176), .out0(new_n179));
  aoai13aa1n12x5               g084(.a(new_n179), .b(new_n130), .c(new_n112), .d(new_n120), .o1(new_n180));
  inv000aa1n02x5               g085(.a(new_n167), .o1(new_n181));
  nano22aa1n03x7               g086(.a(new_n163), .b(new_n178), .c(new_n181), .out0(new_n182));
  nano23aa1n06x5               g087(.a(new_n164), .b(new_n171), .c(new_n181), .d(new_n168), .out0(new_n183));
  inv000aa1d42x5               g088(.a(new_n169), .o1(new_n184));
  aob012aa1n02x5               g089(.a(new_n184), .b(new_n167), .c(new_n170), .out0(new_n185));
  aoi112aa1n09x5               g090(.a(new_n185), .b(new_n183), .c(new_n152), .d(new_n182), .o1(new_n186));
  nand02aa1d08x5               g091(.a(new_n180), .b(new_n186), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g093(.a(\a[18] ), .o1(new_n189));
  inv040aa1d32x5               g094(.a(\a[17] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\b[16] ), .o1(new_n191));
  oaoi03aa1n03x5               g096(.a(new_n190), .b(new_n191), .c(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  xroi22aa1d06x4               g098(.a(new_n190), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n194));
  nanp02aa1n02x5               g099(.a(new_n191), .b(new_n190), .o1(new_n195));
  oaoi03aa1n12x5               g100(.a(\a[18] ), .b(\b[17] ), .c(new_n195), .o1(new_n196));
  nor042aa1n06x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nand02aa1n06x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  aoai13aa1n06x5               g104(.a(new_n199), .b(new_n196), .c(new_n187), .d(new_n194), .o1(new_n200));
  aoi112aa1n02x5               g105(.a(new_n199), .b(new_n196), .c(new_n187), .d(new_n194), .o1(new_n201));
  norb02aa1n02x7               g106(.a(new_n200), .b(new_n201), .out0(\s[19] ));
  xnrc02aa1n02x5               g107(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nand02aa1d12x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  norb02aa1n02x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  oaoi13aa1n06x5               g112(.a(new_n207), .b(new_n200), .c(\a[19] ), .d(\b[18] ), .o1(new_n208));
  nona22aa1n02x5               g113(.a(new_n200), .b(new_n206), .c(new_n197), .out0(new_n209));
  norb02aa1n03x4               g114(.a(new_n209), .b(new_n208), .out0(\s[20] ));
  nano23aa1n09x5               g115(.a(new_n197), .b(new_n204), .c(new_n205), .d(new_n198), .out0(new_n211));
  nanp02aa1n02x5               g116(.a(new_n194), .b(new_n211), .o1(new_n212));
  oai022aa1n02x7               g117(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n213));
  oaib12aa1n09x5               g118(.a(new_n213), .b(new_n189), .c(\b[17] ), .out0(new_n214));
  nona23aa1d18x5               g119(.a(new_n205), .b(new_n198), .c(new_n197), .d(new_n204), .out0(new_n215));
  aoi012aa1n12x5               g120(.a(new_n204), .b(new_n197), .c(new_n205), .o1(new_n216));
  oai012aa1d24x5               g121(.a(new_n216), .b(new_n215), .c(new_n214), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoai13aa1n06x5               g123(.a(new_n218), .b(new_n212), .c(new_n180), .d(new_n186), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[21] ), .b(\b[20] ), .out0(new_n222));
  xorc02aa1n02x5               g127(.a(\a[22] ), .b(\b[21] ), .out0(new_n223));
  aoai13aa1n03x5               g128(.a(new_n223), .b(new_n221), .c(new_n219), .d(new_n222), .o1(new_n224));
  aoi112aa1n02x5               g129(.a(new_n221), .b(new_n223), .c(new_n219), .d(new_n222), .o1(new_n225));
  norb02aa1n03x4               g130(.a(new_n224), .b(new_n225), .out0(\s[22] ));
  inv000aa1d42x5               g131(.a(\a[21] ), .o1(new_n227));
  inv000aa1d42x5               g132(.a(\a[22] ), .o1(new_n228));
  xroi22aa1d06x4               g133(.a(new_n227), .b(\b[20] ), .c(new_n228), .d(\b[21] ), .out0(new_n229));
  nand03aa1n02x5               g134(.a(new_n229), .b(new_n194), .c(new_n211), .o1(new_n230));
  inv000aa1d42x5               g135(.a(\b[21] ), .o1(new_n231));
  oaoi03aa1n12x5               g136(.a(new_n228), .b(new_n231), .c(new_n221), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoi012aa1n02x5               g138(.a(new_n233), .b(new_n217), .c(new_n229), .o1(new_n234));
  aoai13aa1n06x5               g139(.a(new_n234), .b(new_n230), .c(new_n180), .d(new_n186), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  xorc02aa1n03x5               g142(.a(\a[23] ), .b(\b[22] ), .out0(new_n238));
  xorc02aa1n02x5               g143(.a(\a[24] ), .b(\b[23] ), .out0(new_n239));
  aoai13aa1n03x5               g144(.a(new_n239), .b(new_n237), .c(new_n235), .d(new_n238), .o1(new_n240));
  aoi112aa1n02x5               g145(.a(new_n237), .b(new_n239), .c(new_n235), .d(new_n238), .o1(new_n241));
  norb02aa1n03x4               g146(.a(new_n240), .b(new_n241), .out0(\s[24] ));
  inv020aa1n02x5               g147(.a(new_n216), .o1(new_n243));
  aoai13aa1n06x5               g148(.a(new_n229), .b(new_n243), .c(new_n211), .d(new_n196), .o1(new_n244));
  and002aa1n02x5               g149(.a(new_n239), .b(new_n238), .o(new_n245));
  inv000aa1n02x5               g150(.a(new_n245), .o1(new_n246));
  orn002aa1n02x5               g151(.a(\a[23] ), .b(\b[22] ), .o(new_n247));
  oao003aa1n02x5               g152(.a(\a[24] ), .b(\b[23] ), .c(new_n247), .carry(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n246), .c(new_n244), .d(new_n232), .o1(new_n249));
  nano32aa1n02x4               g154(.a(new_n246), .b(new_n229), .c(new_n194), .d(new_n211), .out0(new_n250));
  xorc02aa1n12x5               g155(.a(\a[25] ), .b(\b[24] ), .out0(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n249), .c(new_n187), .d(new_n250), .o1(new_n252));
  aoi112aa1n02x5               g157(.a(new_n249), .b(new_n251), .c(new_n187), .d(new_n250), .o1(new_n253));
  norb02aa1n02x7               g158(.a(new_n252), .b(new_n253), .out0(\s[25] ));
  nor042aa1n03x5               g159(.a(\b[24] ), .b(\a[25] ), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  tech160nm_fixorc02aa1n05x5   g161(.a(\a[26] ), .b(\b[25] ), .out0(new_n257));
  aobi12aa1n06x5               g162(.a(new_n257), .b(new_n252), .c(new_n256), .out0(new_n258));
  nona22aa1n06x5               g163(.a(new_n252), .b(new_n257), .c(new_n255), .out0(new_n259));
  norb02aa1n03x4               g164(.a(new_n259), .b(new_n258), .out0(\s[26] ));
  oab012aa1n03x5               g165(.a(new_n111), .b(new_n110), .c(new_n105), .out0(new_n261));
  nanb03aa1n03x5               g166(.a(new_n117), .b(new_n119), .c(new_n118), .out0(new_n262));
  nanp02aa1n02x5               g167(.a(new_n129), .b(new_n121), .o1(new_n263));
  oaoi03aa1n02x5               g168(.a(new_n122), .b(new_n123), .c(new_n263), .o1(new_n264));
  tech160nm_fioai012aa1n05x5   g169(.a(new_n264), .b(new_n261), .c(new_n262), .o1(new_n265));
  nanp02aa1n02x5               g170(.a(new_n152), .b(new_n182), .o1(new_n266));
  nona22aa1n03x5               g171(.a(new_n266), .b(new_n185), .c(new_n183), .out0(new_n267));
  and002aa1n06x5               g172(.a(new_n257), .b(new_n251), .o(new_n268));
  nano22aa1n03x7               g173(.a(new_n230), .b(new_n245), .c(new_n268), .out0(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n267), .c(new_n265), .d(new_n179), .o1(new_n270));
  oao003aa1n02x5               g175(.a(\a[26] ), .b(\b[25] ), .c(new_n256), .carry(new_n271));
  aobi12aa1n09x5               g176(.a(new_n271), .b(new_n249), .c(new_n268), .out0(new_n272));
  xorc02aa1n12x5               g177(.a(\a[27] ), .b(\b[26] ), .out0(new_n273));
  xnbna2aa1n03x5               g178(.a(new_n273), .b(new_n272), .c(new_n270), .out0(\s[27] ));
  norp02aa1n02x5               g179(.a(\b[26] ), .b(\a[27] ), .o1(new_n275));
  inv040aa1n03x5               g180(.a(new_n275), .o1(new_n276));
  inv000aa1n02x5               g181(.a(new_n269), .o1(new_n277));
  aoi012aa1n06x5               g182(.a(new_n277), .b(new_n180), .c(new_n186), .o1(new_n278));
  aoai13aa1n03x5               g183(.a(new_n245), .b(new_n233), .c(new_n217), .d(new_n229), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n268), .o1(new_n280));
  aoai13aa1n04x5               g185(.a(new_n271), .b(new_n280), .c(new_n279), .d(new_n248), .o1(new_n281));
  oaih12aa1n02x5               g186(.a(new_n273), .b(new_n281), .c(new_n278), .o1(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[27] ), .b(\a[28] ), .out0(new_n283));
  tech160nm_fiaoi012aa1n02p5x5 g188(.a(new_n283), .b(new_n282), .c(new_n276), .o1(new_n284));
  aobi12aa1n03x5               g189(.a(new_n273), .b(new_n272), .c(new_n270), .out0(new_n285));
  nano22aa1n03x5               g190(.a(new_n285), .b(new_n276), .c(new_n283), .out0(new_n286));
  norp02aa1n03x5               g191(.a(new_n284), .b(new_n286), .o1(\s[28] ));
  xnrc02aa1n02x5               g192(.a(\b[28] ), .b(\a[29] ), .out0(new_n288));
  norb02aa1n02x5               g193(.a(new_n273), .b(new_n283), .out0(new_n289));
  oaih12aa1n02x5               g194(.a(new_n289), .b(new_n281), .c(new_n278), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n276), .carry(new_n291));
  aoi012aa1n03x5               g196(.a(new_n288), .b(new_n290), .c(new_n291), .o1(new_n292));
  aobi12aa1n02x7               g197(.a(new_n289), .b(new_n272), .c(new_n270), .out0(new_n293));
  nano22aa1n03x5               g198(.a(new_n293), .b(new_n288), .c(new_n291), .out0(new_n294));
  nor002aa1n02x5               g199(.a(new_n292), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g201(.a(\b[29] ), .b(\a[30] ), .out0(new_n297));
  norb03aa1n02x5               g202(.a(new_n273), .b(new_n288), .c(new_n283), .out0(new_n298));
  oaih12aa1n02x5               g203(.a(new_n298), .b(new_n281), .c(new_n278), .o1(new_n299));
  oao003aa1n02x5               g204(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .carry(new_n300));
  aoi012aa1n03x5               g205(.a(new_n297), .b(new_n299), .c(new_n300), .o1(new_n301));
  aobi12aa1n06x5               g206(.a(new_n298), .b(new_n272), .c(new_n270), .out0(new_n302));
  nano22aa1n03x5               g207(.a(new_n302), .b(new_n297), .c(new_n300), .out0(new_n303));
  nor002aa1n02x5               g208(.a(new_n301), .b(new_n303), .o1(\s[30] ));
  norb02aa1n02x5               g209(.a(new_n298), .b(new_n297), .out0(new_n305));
  oaih12aa1n02x5               g210(.a(new_n305), .b(new_n281), .c(new_n278), .o1(new_n306));
  oao003aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .c(new_n300), .carry(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[30] ), .b(\a[31] ), .out0(new_n308));
  tech160nm_fiaoi012aa1n02p5x5 g213(.a(new_n308), .b(new_n306), .c(new_n307), .o1(new_n309));
  aobi12aa1n06x5               g214(.a(new_n305), .b(new_n272), .c(new_n270), .out0(new_n310));
  nano22aa1n03x5               g215(.a(new_n310), .b(new_n307), .c(new_n308), .out0(new_n311));
  norp02aa1n03x5               g216(.a(new_n309), .b(new_n311), .o1(\s[31] ));
  xnrb03aa1n02x5               g217(.a(new_n105), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g218(.a(\a[3] ), .b(\b[2] ), .c(new_n105), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g220(.a(new_n112), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoai13aa1n02x5               g221(.a(new_n119), .b(new_n115), .c(new_n112), .d(new_n116), .o1(new_n317));
  aoi112aa1n02x5               g222(.a(new_n115), .b(new_n119), .c(new_n112), .d(new_n116), .o1(new_n318));
  norb02aa1n02x5               g223(.a(new_n317), .b(new_n318), .out0(\s[6] ));
  nona22aa1n02x4               g224(.a(new_n317), .b(new_n128), .c(new_n127), .out0(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g226(.a(new_n113), .b(new_n320), .c(new_n114), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[7] ), .c(new_n122), .out0(\s[8] ));
  xorb03aa1n02x5               g228(.a(new_n265), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


