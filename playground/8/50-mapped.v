// Benchmark "adder" written by ABC on Wed Jul 17 16:30:45 2024

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
    new_n133, new_n135, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n159, new_n160, new_n161, new_n162, new_n163, new_n165, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n179, new_n180, new_n181,
    new_n182, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n212, new_n213, new_n214,
    new_n215, new_n216, new_n217, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n228, new_n229, new_n230,
    new_n231, new_n232, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n314, new_n317, new_n319, new_n321;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[2] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[1] ), .o1(new_n98));
  nand22aa1n03x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  oaoi03aa1n06x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  nor002aa1d24x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand22aa1n04x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor002aa1n12x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n04x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n09x5               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  aoi012aa1n06x5               g010(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n106));
  oai012aa1n09x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  nor022aa1n04x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  nand42aa1n02x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nor022aa1n16x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nand22aa1n04x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nona23aa1n02x4               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  xorc02aa1n02x5               g017(.a(\a[7] ), .b(\b[6] ), .out0(new_n113));
  tech160nm_fixnrc02aa1n02p5x5 g018(.a(\b[7] ), .b(\a[8] ), .out0(new_n114));
  norb03aa1n03x5               g019(.a(new_n113), .b(new_n112), .c(new_n114), .out0(new_n115));
  inv000aa1d42x5               g020(.a(\a[8] ), .o1(new_n116));
  inv000aa1d42x5               g021(.a(\a[7] ), .o1(new_n117));
  inv000aa1d42x5               g022(.a(\a[5] ), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\b[4] ), .o1(new_n119));
  aoai13aa1n06x5               g024(.a(new_n111), .b(new_n110), .c(new_n118), .d(new_n119), .o1(new_n120));
  oaib12aa1n06x5               g025(.a(new_n120), .b(\b[6] ), .c(new_n117), .out0(new_n121));
  aoi022aa1n02x5               g026(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n122));
  nanp02aa1n02x5               g027(.a(new_n121), .b(new_n122), .o1(new_n123));
  oaib12aa1n09x5               g028(.a(new_n123), .b(\b[7] ), .c(new_n116), .out0(new_n124));
  tech160nm_fiaoi012aa1n02p5x5 g029(.a(new_n124), .b(new_n107), .c(new_n115), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .c(new_n125), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  tech160nm_fixnrc02aa1n02p5x5 g032(.a(\b[9] ), .b(\a[10] ), .out0(new_n128));
  xnrc02aa1n02x5               g033(.a(\b[8] ), .b(\a[9] ), .out0(new_n129));
  nor002aa1n02x5               g034(.a(new_n129), .b(new_n128), .o1(new_n130));
  oaih22aa1d12x5               g035(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n131));
  aob012aa1n09x5               g036(.a(new_n131), .b(\b[9] ), .c(\a[10] ), .out0(new_n132));
  oaib12aa1n06x5               g037(.a(new_n132), .b(new_n125), .c(new_n130), .out0(new_n133));
  xorb03aa1n02x5               g038(.a(new_n133), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1n20x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand02aa1n06x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  tech160nm_fiaoi012aa1n05x5   g041(.a(new_n135), .b(new_n133), .c(new_n136), .o1(new_n137));
  xnrb03aa1n03x5               g042(.a(new_n137), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor022aa1n16x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanp02aa1n04x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nona23aa1d16x5               g045(.a(new_n140), .b(new_n136), .c(new_n135), .d(new_n139), .out0(new_n141));
  norp03aa1n02x5               g046(.a(new_n141), .b(new_n129), .c(new_n128), .o1(new_n142));
  aoai13aa1n06x5               g047(.a(new_n142), .b(new_n124), .c(new_n107), .d(new_n115), .o1(new_n143));
  tech160nm_fioai012aa1n03p5x5 g048(.a(new_n140), .b(new_n139), .c(new_n135), .o1(new_n144));
  oai012aa1d24x5               g049(.a(new_n144), .b(new_n141), .c(new_n132), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(new_n143), .b(new_n146), .o1(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1n12x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  tech160nm_finand02aa1n03p5x5 g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  aoi012aa1n02x5               g055(.a(new_n149), .b(new_n147), .c(new_n150), .o1(new_n151));
  xnrb03aa1n02x5               g056(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n08x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nanp02aa1n04x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nona23aa1n03x5               g059(.a(new_n154), .b(new_n150), .c(new_n149), .d(new_n153), .out0(new_n155));
  oai012aa1n02x7               g060(.a(new_n154), .b(new_n153), .c(new_n149), .o1(new_n156));
  aoai13aa1n02x7               g061(.a(new_n156), .b(new_n155), .c(new_n143), .d(new_n146), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  xorc02aa1n02x5               g064(.a(\a[15] ), .b(\b[14] ), .out0(new_n160));
  tech160nm_fixnrc02aa1n04x5   g065(.a(\b[15] ), .b(\a[16] ), .out0(new_n161));
  aoai13aa1n02x5               g066(.a(new_n161), .b(new_n159), .c(new_n157), .d(new_n160), .o1(new_n162));
  aoi112aa1n02x5               g067(.a(new_n159), .b(new_n161), .c(new_n157), .d(new_n160), .o1(new_n163));
  nanb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(\s[16] ));
  nano23aa1n03x5               g069(.a(new_n135), .b(new_n139), .c(new_n140), .d(new_n136), .out0(new_n165));
  nano23aa1n02x5               g070(.a(new_n149), .b(new_n153), .c(new_n154), .d(new_n150), .out0(new_n166));
  xorc02aa1n02x5               g071(.a(\a[16] ), .b(\b[15] ), .out0(new_n167));
  nanp03aa1n02x5               g072(.a(new_n166), .b(new_n160), .c(new_n167), .o1(new_n168));
  nano22aa1n03x7               g073(.a(new_n168), .b(new_n130), .c(new_n165), .out0(new_n169));
  aoai13aa1n06x5               g074(.a(new_n169), .b(new_n124), .c(new_n107), .d(new_n115), .o1(new_n170));
  xnrc02aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .out0(new_n171));
  nor043aa1n03x5               g076(.a(new_n155), .b(new_n171), .c(new_n161), .o1(new_n172));
  orn002aa1n02x5               g077(.a(\a[15] ), .b(\b[14] ), .o(new_n173));
  oao003aa1n02x5               g078(.a(\a[16] ), .b(\b[15] ), .c(new_n173), .carry(new_n174));
  oai013aa1n03x5               g079(.a(new_n174), .b(new_n171), .c(new_n161), .d(new_n156), .o1(new_n175));
  aoi012aa1d18x5               g080(.a(new_n175), .b(new_n145), .c(new_n172), .o1(new_n176));
  nanp02aa1n06x5               g081(.a(new_n170), .b(new_n176), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g083(.a(\a[18] ), .o1(new_n179));
  inv040aa1d32x5               g084(.a(\a[17] ), .o1(new_n180));
  inv040aa1d28x5               g085(.a(\b[16] ), .o1(new_n181));
  oaoi03aa1n02x5               g086(.a(new_n180), .b(new_n181), .c(new_n177), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[17] ), .c(new_n179), .out0(\s[18] ));
  xroi22aa1d06x4               g088(.a(new_n180), .b(\b[16] ), .c(new_n179), .d(\b[17] ), .out0(new_n184));
  tech160nm_finand02aa1n03p5x5 g089(.a(new_n181), .b(new_n180), .o1(new_n185));
  oaoi03aa1n12x5               g090(.a(\a[18] ), .b(\b[17] ), .c(new_n185), .o1(new_n186));
  nor042aa1n12x5               g091(.a(\b[18] ), .b(\a[19] ), .o1(new_n187));
  nand02aa1n08x5               g092(.a(\b[18] ), .b(\a[19] ), .o1(new_n188));
  nanb02aa1n02x5               g093(.a(new_n187), .b(new_n188), .out0(new_n189));
  inv000aa1d42x5               g094(.a(new_n189), .o1(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n186), .c(new_n177), .d(new_n184), .o1(new_n191));
  aoi112aa1n02x5               g096(.a(new_n190), .b(new_n186), .c(new_n177), .d(new_n184), .o1(new_n192));
  norb02aa1n02x5               g097(.a(new_n191), .b(new_n192), .out0(\s[19] ));
  xnrc02aa1n02x5               g098(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n09x5               g099(.a(\b[19] ), .b(\a[20] ), .o1(new_n195));
  nand02aa1d10x5               g100(.a(\b[19] ), .b(\a[20] ), .o1(new_n196));
  nanb02aa1n02x5               g101(.a(new_n195), .b(new_n196), .out0(new_n197));
  tech160nm_fioai012aa1n03p5x5 g102(.a(new_n191), .b(\b[18] ), .c(\a[19] ), .o1(new_n198));
  nand02aa1n02x5               g103(.a(new_n198), .b(new_n197), .o1(new_n199));
  nona22aa1n02x5               g104(.a(new_n191), .b(new_n197), .c(new_n187), .out0(new_n200));
  nanp02aa1n03x5               g105(.a(new_n199), .b(new_n200), .o1(\s[20] ));
  nano23aa1n09x5               g106(.a(new_n187), .b(new_n195), .c(new_n196), .d(new_n188), .out0(new_n202));
  nanp02aa1n02x5               g107(.a(new_n184), .b(new_n202), .o1(new_n203));
  oai022aa1n02x5               g108(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n204));
  oaib12aa1n06x5               g109(.a(new_n204), .b(new_n179), .c(\b[17] ), .out0(new_n205));
  nona23aa1n09x5               g110(.a(new_n196), .b(new_n188), .c(new_n187), .d(new_n195), .out0(new_n206));
  oaih12aa1n06x5               g111(.a(new_n196), .b(new_n195), .c(new_n187), .o1(new_n207));
  oai012aa1n12x5               g112(.a(new_n207), .b(new_n206), .c(new_n205), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  aoai13aa1n04x5               g114(.a(new_n209), .b(new_n203), .c(new_n170), .d(new_n176), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g116(.a(\b[20] ), .b(\a[21] ), .o1(new_n212));
  xorc02aa1n02x5               g117(.a(\a[21] ), .b(\b[20] ), .out0(new_n213));
  xorc02aa1n02x5               g118(.a(\a[22] ), .b(\b[21] ), .out0(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  aoai13aa1n02x5               g120(.a(new_n215), .b(new_n212), .c(new_n210), .d(new_n213), .o1(new_n216));
  aoi112aa1n03x5               g121(.a(new_n212), .b(new_n215), .c(new_n210), .d(new_n213), .o1(new_n217));
  nanb02aa1n02x5               g122(.a(new_n217), .b(new_n216), .out0(\s[22] ));
  inv000aa1d42x5               g123(.a(\a[21] ), .o1(new_n219));
  inv040aa1d32x5               g124(.a(\a[22] ), .o1(new_n220));
  xroi22aa1d06x4               g125(.a(new_n219), .b(\b[20] ), .c(new_n220), .d(\b[21] ), .out0(new_n221));
  nanp03aa1n02x5               g126(.a(new_n221), .b(new_n184), .c(new_n202), .o1(new_n222));
  inv000aa1d42x5               g127(.a(\b[21] ), .o1(new_n223));
  oao003aa1n06x5               g128(.a(new_n220), .b(new_n223), .c(new_n212), .carry(new_n224));
  aoi012aa1n02x5               g129(.a(new_n224), .b(new_n208), .c(new_n221), .o1(new_n225));
  aoai13aa1n04x5               g130(.a(new_n225), .b(new_n222), .c(new_n170), .d(new_n176), .o1(new_n226));
  xorb03aa1n02x5               g131(.a(new_n226), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g132(.a(\b[22] ), .b(\a[23] ), .o1(new_n228));
  xorc02aa1n12x5               g133(.a(\a[23] ), .b(\b[22] ), .out0(new_n229));
  tech160nm_fixnrc02aa1n05x5   g134(.a(\b[23] ), .b(\a[24] ), .out0(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n228), .c(new_n226), .d(new_n229), .o1(new_n231));
  aoi112aa1n03x5               g136(.a(new_n228), .b(new_n230), .c(new_n226), .d(new_n229), .o1(new_n232));
  nanb02aa1n03x5               g137(.a(new_n232), .b(new_n231), .out0(\s[24] ));
  oao003aa1n02x5               g138(.a(new_n97), .b(new_n98), .c(new_n99), .carry(new_n234));
  nano23aa1n03x7               g139(.a(new_n101), .b(new_n103), .c(new_n104), .d(new_n102), .out0(new_n235));
  aobi12aa1n03x7               g140(.a(new_n106), .b(new_n235), .c(new_n234), .out0(new_n236));
  nanb02aa1n02x5               g141(.a(new_n108), .b(new_n109), .out0(new_n237));
  norb02aa1n03x5               g142(.a(new_n111), .b(new_n110), .out0(new_n238));
  nona23aa1n03x5               g143(.a(new_n113), .b(new_n238), .c(new_n114), .d(new_n237), .out0(new_n239));
  aboi22aa1n03x5               g144(.a(\b[7] ), .b(new_n116), .c(new_n121), .d(new_n122), .out0(new_n240));
  oaih12aa1n02x5               g145(.a(new_n240), .b(new_n236), .c(new_n239), .o1(new_n241));
  inv020aa1n04x5               g146(.a(new_n176), .o1(new_n242));
  norb02aa1n02x7               g147(.a(new_n229), .b(new_n230), .out0(new_n243));
  nano22aa1n02x4               g148(.a(new_n203), .b(new_n243), .c(new_n221), .out0(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n242), .c(new_n241), .d(new_n169), .o1(new_n245));
  inv020aa1n03x5               g150(.a(new_n207), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n221), .b(new_n246), .c(new_n202), .d(new_n186), .o1(new_n247));
  inv030aa1n02x5               g152(.a(new_n224), .o1(new_n248));
  inv030aa1n02x5               g153(.a(new_n243), .o1(new_n249));
  oai022aa1n02x5               g154(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n250));
  aob012aa1n02x5               g155(.a(new_n250), .b(\b[23] ), .c(\a[24] ), .out0(new_n251));
  aoai13aa1n12x5               g156(.a(new_n251), .b(new_n249), .c(new_n247), .d(new_n248), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n252), .o1(new_n253));
  xorc02aa1n12x5               g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  xnbna2aa1n03x5               g159(.a(new_n254), .b(new_n245), .c(new_n253), .out0(\s[25] ));
  nanp02aa1n02x5               g160(.a(new_n245), .b(new_n253), .o1(new_n256));
  norp02aa1n02x5               g161(.a(\b[24] ), .b(\a[25] ), .o1(new_n257));
  nor002aa1n03x5               g162(.a(\b[25] ), .b(\a[26] ), .o1(new_n258));
  nand42aa1n06x5               g163(.a(\b[25] ), .b(\a[26] ), .o1(new_n259));
  norb02aa1n06x4               g164(.a(new_n259), .b(new_n258), .out0(new_n260));
  inv040aa1n03x5               g165(.a(new_n260), .o1(new_n261));
  aoai13aa1n02x5               g166(.a(new_n261), .b(new_n257), .c(new_n256), .d(new_n254), .o1(new_n262));
  aoai13aa1n02x5               g167(.a(new_n254), .b(new_n252), .c(new_n177), .d(new_n244), .o1(new_n263));
  nona22aa1n02x4               g168(.a(new_n263), .b(new_n261), .c(new_n257), .out0(new_n264));
  nanp02aa1n02x5               g169(.a(new_n262), .b(new_n264), .o1(\s[26] ));
  norb02aa1n12x5               g170(.a(new_n254), .b(new_n261), .out0(new_n266));
  nano22aa1n06x5               g171(.a(new_n222), .b(new_n243), .c(new_n266), .out0(new_n267));
  aoai13aa1n06x5               g172(.a(new_n267), .b(new_n242), .c(new_n241), .d(new_n169), .o1(new_n268));
  oai012aa1n02x5               g173(.a(new_n259), .b(new_n258), .c(new_n257), .o1(new_n269));
  aobi12aa1n12x5               g174(.a(new_n269), .b(new_n252), .c(new_n266), .out0(new_n270));
  xorc02aa1n02x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  xnbna2aa1n03x5               g176(.a(new_n271), .b(new_n270), .c(new_n268), .out0(\s[27] ));
  nand42aa1n03x5               g177(.a(new_n270), .b(new_n268), .o1(new_n273));
  norp02aa1n02x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  norp02aa1n02x5               g179(.a(\b[27] ), .b(\a[28] ), .o1(new_n275));
  nand42aa1n03x5               g180(.a(\b[27] ), .b(\a[28] ), .o1(new_n276));
  norb02aa1n03x5               g181(.a(new_n276), .b(new_n275), .out0(new_n277));
  inv000aa1d42x5               g182(.a(new_n277), .o1(new_n278));
  aoai13aa1n03x5               g183(.a(new_n278), .b(new_n274), .c(new_n273), .d(new_n271), .o1(new_n279));
  aobi12aa1n06x5               g184(.a(new_n267), .b(new_n170), .c(new_n176), .out0(new_n280));
  aoai13aa1n03x5               g185(.a(new_n243), .b(new_n224), .c(new_n208), .d(new_n221), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n266), .o1(new_n282));
  aoai13aa1n04x5               g187(.a(new_n269), .b(new_n282), .c(new_n281), .d(new_n251), .o1(new_n283));
  oaih12aa1n02x5               g188(.a(new_n271), .b(new_n283), .c(new_n280), .o1(new_n284));
  nona22aa1n02x5               g189(.a(new_n284), .b(new_n278), .c(new_n274), .out0(new_n285));
  nanp02aa1n03x5               g190(.a(new_n279), .b(new_n285), .o1(\s[28] ));
  norb02aa1n02x5               g191(.a(new_n271), .b(new_n278), .out0(new_n287));
  oaih12aa1n02x5               g192(.a(new_n287), .b(new_n283), .c(new_n280), .o1(new_n288));
  oai012aa1n02x5               g193(.a(new_n276), .b(new_n275), .c(new_n274), .o1(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  tech160nm_fiaoi012aa1n02p5x5 g195(.a(new_n290), .b(new_n288), .c(new_n289), .o1(new_n291));
  aobi12aa1n02x7               g196(.a(new_n287), .b(new_n270), .c(new_n268), .out0(new_n292));
  nano22aa1n03x5               g197(.a(new_n292), .b(new_n289), .c(new_n290), .out0(new_n293));
  norp02aa1n03x5               g198(.a(new_n291), .b(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g199(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g200(.a(new_n290), .b(new_n271), .c(new_n277), .out0(new_n296));
  oaih12aa1n02x5               g201(.a(new_n296), .b(new_n283), .c(new_n280), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[29] ), .b(\a[30] ), .out0(new_n299));
  tech160nm_fiaoi012aa1n02p5x5 g204(.a(new_n299), .b(new_n297), .c(new_n298), .o1(new_n300));
  aobi12aa1n02x7               g205(.a(new_n296), .b(new_n270), .c(new_n268), .out0(new_n301));
  nano22aa1n03x5               g206(.a(new_n301), .b(new_n298), .c(new_n299), .out0(new_n302));
  norp02aa1n03x5               g207(.a(new_n300), .b(new_n302), .o1(\s[30] ));
  nano23aa1n02x4               g208(.a(new_n299), .b(new_n290), .c(new_n271), .d(new_n277), .out0(new_n304));
  oaih12aa1n02x5               g209(.a(new_n304), .b(new_n283), .c(new_n280), .o1(new_n305));
  oao003aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .c(new_n298), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  tech160nm_fiaoi012aa1n02p5x5 g212(.a(new_n307), .b(new_n305), .c(new_n306), .o1(new_n308));
  aobi12aa1n02x7               g213(.a(new_n304), .b(new_n270), .c(new_n268), .out0(new_n309));
  nano22aa1n03x5               g214(.a(new_n309), .b(new_n306), .c(new_n307), .out0(new_n310));
  norp02aa1n03x5               g215(.a(new_n308), .b(new_n310), .o1(\s[31] ));
  xnrb03aa1n02x5               g216(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  inv000aa1d42x5               g217(.a(new_n101), .o1(new_n313));
  aoi122aa1n02x5               g218(.a(new_n103), .b(new_n313), .c(new_n102), .d(new_n234), .e(new_n104), .o1(new_n314));
  aoi012aa1n02x5               g219(.a(new_n314), .b(new_n313), .c(new_n107), .o1(\s[4] ));
  xorb03aa1n02x5               g220(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g221(.a(new_n118), .b(new_n119), .c(new_n107), .o1(new_n317));
  xnrc02aa1n02x5               g222(.a(new_n317), .b(new_n238), .out0(\s[6] ));
  aob012aa1n02x5               g223(.a(new_n111), .b(new_n317), .c(new_n238), .out0(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[6] ), .c(new_n117), .out0(\s[7] ));
  oaoi03aa1n02x5               g225(.a(\a[7] ), .b(\b[6] ), .c(new_n319), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g227(.a(new_n241), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule

