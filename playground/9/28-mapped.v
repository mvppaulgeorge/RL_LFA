// Benchmark "adder" written by ABC on Wed Jul 17 16:48:03 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n135, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n230,
    new_n231, new_n232, new_n233, new_n234, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n248, new_n249, new_n250, new_n251, new_n252, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n305, new_n306, new_n307, new_n310, new_n311, new_n312,
    new_n314, new_n316;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d24x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand22aa1n03x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  tech160nm_fiaoi012aa1n04x5   g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nor002aa1d32x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor002aa1n12x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n09x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  tech160nm_fiaoi012aa1n03p5x5 g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  oai012aa1n12x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  nor002aa1n06x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand22aa1n03x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor022aa1n04x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand02aa1d04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n09x5               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nor043aa1n06x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  aoi112aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n118));
  nano23aa1n06x5               g023(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n119));
  and002aa1n02x7               g024(.a(\b[5] ), .b(\a[6] ), .o(new_n120));
  oa0022aa1n03x5               g025(.a(\a[6] ), .b(\b[5] ), .c(\a[5] ), .d(\b[4] ), .o(new_n121));
  nona22aa1n09x5               g026(.a(new_n119), .b(new_n120), .c(new_n121), .out0(new_n122));
  nona22aa1n12x5               g027(.a(new_n122), .b(new_n118), .c(new_n110), .out0(new_n123));
  xorc02aa1n12x5               g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  aoai13aa1n04x5               g029(.a(new_n124), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n125));
  xorc02aa1n12x5               g030(.a(\a[10] ), .b(\b[9] ), .out0(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n126), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g032(.a(\a[10] ), .o1(new_n128));
  inv000aa1d42x5               g033(.a(\b[9] ), .o1(new_n129));
  aoi012aa1n02x5               g034(.a(new_n97), .b(new_n128), .c(new_n129), .o1(new_n130));
  aoi022aa1n06x5               g035(.a(new_n125), .b(new_n130), .c(\b[9] ), .d(\a[10] ), .o1(new_n131));
  xorb03aa1n02x5               g036(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nand42aa1d28x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nor002aa1d32x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  aoi012aa1n03x5               g039(.a(new_n134), .b(new_n131), .c(new_n133), .o1(new_n135));
  xnrb03aa1n03x5               g040(.a(new_n135), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nand22aa1n03x5               g041(.a(new_n109), .b(new_n117), .o1(new_n137));
  inv000aa1n02x5               g042(.a(new_n120), .o1(new_n138));
  inv000aa1n02x5               g043(.a(new_n121), .o1(new_n139));
  aoi113aa1n03x7               g044(.a(new_n110), .b(new_n118), .c(new_n119), .d(new_n139), .e(new_n138), .o1(new_n140));
  nor002aa1d32x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand42aa1n20x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nano23aa1d15x5               g047(.a(new_n141), .b(new_n134), .c(new_n142), .d(new_n133), .out0(new_n143));
  nand23aa1d12x5               g048(.a(new_n143), .b(new_n124), .c(new_n126), .o1(new_n144));
  nona23aa1n09x5               g049(.a(new_n133), .b(new_n142), .c(new_n141), .d(new_n134), .out0(new_n145));
  tech160nm_fioai012aa1n03p5x5 g050(.a(new_n142), .b(new_n141), .c(new_n134), .o1(new_n146));
  oaoi03aa1n09x5               g051(.a(new_n128), .b(new_n129), .c(new_n97), .o1(new_n147));
  oai012aa1n18x5               g052(.a(new_n146), .b(new_n145), .c(new_n147), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  aoai13aa1n03x5               g054(.a(new_n149), .b(new_n144), .c(new_n140), .d(new_n137), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n12x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nanp02aa1n06x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  aoi012aa1n03x5               g058(.a(new_n152), .b(new_n150), .c(new_n153), .o1(new_n154));
  xnrb03aa1n03x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  inv000aa1d42x5               g060(.a(new_n144), .o1(new_n156));
  aoai13aa1n04x5               g061(.a(new_n156), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n157));
  norp02aa1n12x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nand42aa1n04x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nona23aa1n09x5               g064(.a(new_n159), .b(new_n153), .c(new_n152), .d(new_n158), .out0(new_n160));
  tech160nm_fioai012aa1n04x5   g065(.a(new_n159), .b(new_n158), .c(new_n152), .o1(new_n161));
  aoai13aa1n04x5               g066(.a(new_n161), .b(new_n160), .c(new_n157), .d(new_n149), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  xnrc02aa1n12x5               g069(.a(\b[14] ), .b(\a[15] ), .out0(new_n165));
  inv040aa1n02x5               g070(.a(new_n165), .o1(new_n166));
  xnrc02aa1n12x5               g071(.a(\b[15] ), .b(\a[16] ), .out0(new_n167));
  inv040aa1n02x5               g072(.a(new_n167), .o1(new_n168));
  aoai13aa1n03x5               g073(.a(new_n168), .b(new_n164), .c(new_n162), .d(new_n166), .o1(new_n169));
  aoi112aa1n02x5               g074(.a(new_n164), .b(new_n168), .c(new_n162), .d(new_n166), .o1(new_n170));
  norb02aa1n02x7               g075(.a(new_n169), .b(new_n170), .out0(\s[16] ));
  nano23aa1n06x5               g076(.a(new_n152), .b(new_n158), .c(new_n159), .d(new_n153), .out0(new_n172));
  nano32aa1d12x5               g077(.a(new_n144), .b(new_n168), .c(new_n172), .d(new_n166), .out0(new_n173));
  aoai13aa1n12x5               g078(.a(new_n173), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n174));
  nor043aa1n02x5               g079(.a(new_n160), .b(new_n165), .c(new_n167), .o1(new_n175));
  aoi112aa1n02x5               g080(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n176));
  nor042aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  inv000aa1d42x5               g082(.a(new_n177), .o1(new_n178));
  oai013aa1n03x5               g083(.a(new_n178), .b(new_n165), .c(new_n167), .d(new_n161), .o1(new_n179));
  aoi112aa1n09x5               g084(.a(new_n179), .b(new_n176), .c(new_n148), .d(new_n175), .o1(new_n180));
  nand02aa1d10x5               g085(.a(new_n174), .b(new_n180), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g087(.a(\a[18] ), .o1(new_n183));
  inv000aa1d42x5               g088(.a(\a[17] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\b[16] ), .o1(new_n185));
  oaoi03aa1n03x5               g090(.a(new_n184), .b(new_n185), .c(new_n181), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  xroi22aa1d04x5               g092(.a(new_n184), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n188));
  oaih22aa1n06x5               g093(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n189));
  oaib12aa1n18x5               g094(.a(new_n189), .b(new_n183), .c(\b[17] ), .out0(new_n190));
  inv000aa1d42x5               g095(.a(new_n190), .o1(new_n191));
  nor042aa1n04x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  nand42aa1n02x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n191), .c(new_n181), .d(new_n188), .o1(new_n195));
  aoi112aa1n02x5               g100(.a(new_n194), .b(new_n191), .c(new_n181), .d(new_n188), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n195), .b(new_n196), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  orn002aa1n02x5               g103(.a(\a[19] ), .b(\b[18] ), .o(new_n199));
  nor022aa1n06x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nand02aa1d04x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  aobi12aa1n06x5               g107(.a(new_n202), .b(new_n195), .c(new_n199), .out0(new_n203));
  nona22aa1n02x5               g108(.a(new_n195), .b(new_n202), .c(new_n192), .out0(new_n204));
  norb02aa1n03x4               g109(.a(new_n204), .b(new_n203), .out0(\s[20] ));
  nano23aa1n06x5               g110(.a(new_n192), .b(new_n200), .c(new_n201), .d(new_n193), .out0(new_n206));
  nanp02aa1n02x5               g111(.a(new_n188), .b(new_n206), .o1(new_n207));
  nona23aa1n09x5               g112(.a(new_n201), .b(new_n193), .c(new_n192), .d(new_n200), .out0(new_n208));
  tech160nm_fiaoi012aa1n04x5   g113(.a(new_n200), .b(new_n192), .c(new_n201), .o1(new_n209));
  oai012aa1n18x5               g114(.a(new_n209), .b(new_n208), .c(new_n190), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  aoai13aa1n06x5               g116(.a(new_n211), .b(new_n207), .c(new_n174), .d(new_n180), .o1(new_n212));
  xorb03aa1n02x5               g117(.a(new_n212), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n04x5               g118(.a(\b[20] ), .b(\a[21] ), .o1(new_n214));
  xorc02aa1n02x5               g119(.a(\a[21] ), .b(\b[20] ), .out0(new_n215));
  xorc02aa1n02x5               g120(.a(\a[22] ), .b(\b[21] ), .out0(new_n216));
  aoai13aa1n03x5               g121(.a(new_n216), .b(new_n214), .c(new_n212), .d(new_n215), .o1(new_n217));
  aoi112aa1n03x5               g122(.a(new_n214), .b(new_n216), .c(new_n212), .d(new_n215), .o1(new_n218));
  norb02aa1n03x4               g123(.a(new_n217), .b(new_n218), .out0(\s[22] ));
  inv000aa1d42x5               g124(.a(\a[21] ), .o1(new_n220));
  inv000aa1d42x5               g125(.a(\a[22] ), .o1(new_n221));
  xroi22aa1d04x5               g126(.a(new_n220), .b(\b[20] ), .c(new_n221), .d(\b[21] ), .out0(new_n222));
  nanp03aa1n02x5               g127(.a(new_n222), .b(new_n188), .c(new_n206), .o1(new_n223));
  inv000aa1d42x5               g128(.a(\b[21] ), .o1(new_n224));
  oaoi03aa1n12x5               g129(.a(new_n221), .b(new_n224), .c(new_n214), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoi012aa1n02x5               g131(.a(new_n226), .b(new_n210), .c(new_n222), .o1(new_n227));
  aoai13aa1n06x5               g132(.a(new_n227), .b(new_n223), .c(new_n174), .d(new_n180), .o1(new_n228));
  xorb03aa1n02x5               g133(.a(new_n228), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g134(.a(\b[22] ), .b(\a[23] ), .o1(new_n230));
  tech160nm_fixorc02aa1n02p5x5 g135(.a(\a[23] ), .b(\b[22] ), .out0(new_n231));
  tech160nm_fixorc02aa1n02p5x5 g136(.a(\a[24] ), .b(\b[23] ), .out0(new_n232));
  aoai13aa1n03x5               g137(.a(new_n232), .b(new_n230), .c(new_n228), .d(new_n231), .o1(new_n233));
  aoi112aa1n02x5               g138(.a(new_n230), .b(new_n232), .c(new_n228), .d(new_n231), .o1(new_n234));
  norb02aa1n02x7               g139(.a(new_n233), .b(new_n234), .out0(\s[24] ));
  and002aa1n06x5               g140(.a(new_n232), .b(new_n231), .o(new_n236));
  inv000aa1n02x5               g141(.a(new_n236), .o1(new_n237));
  nano32aa1n02x4               g142(.a(new_n237), .b(new_n222), .c(new_n188), .d(new_n206), .out0(new_n238));
  inv020aa1n02x5               g143(.a(new_n209), .o1(new_n239));
  aoai13aa1n06x5               g144(.a(new_n222), .b(new_n239), .c(new_n206), .d(new_n191), .o1(new_n240));
  aoi112aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n241));
  oab012aa1n02x4               g146(.a(new_n241), .b(\a[24] ), .c(\b[23] ), .out0(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n237), .c(new_n240), .d(new_n225), .o1(new_n243));
  xorc02aa1n02x5               g148(.a(\a[25] ), .b(\b[24] ), .out0(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n243), .c(new_n181), .d(new_n238), .o1(new_n245));
  aoi112aa1n02x5               g150(.a(new_n244), .b(new_n243), .c(new_n181), .d(new_n238), .o1(new_n246));
  norb02aa1n02x5               g151(.a(new_n245), .b(new_n246), .out0(\s[25] ));
  norp02aa1n02x5               g152(.a(\b[24] ), .b(\a[25] ), .o1(new_n248));
  inv000aa1n02x5               g153(.a(new_n248), .o1(new_n249));
  xorc02aa1n02x5               g154(.a(\a[26] ), .b(\b[25] ), .out0(new_n250));
  aobi12aa1n06x5               g155(.a(new_n250), .b(new_n245), .c(new_n249), .out0(new_n251));
  nona22aa1n02x5               g156(.a(new_n245), .b(new_n250), .c(new_n248), .out0(new_n252));
  norb02aa1n03x4               g157(.a(new_n252), .b(new_n251), .out0(\s[26] ));
  nanp02aa1n03x5               g158(.a(new_n140), .b(new_n137), .o1(new_n254));
  nanp02aa1n02x5               g159(.a(new_n148), .b(new_n175), .o1(new_n255));
  nona22aa1n02x4               g160(.a(new_n255), .b(new_n179), .c(new_n176), .out0(new_n256));
  inv000aa1d42x5               g161(.a(\a[25] ), .o1(new_n257));
  inv040aa1d32x5               g162(.a(\a[26] ), .o1(new_n258));
  xroi22aa1d06x4               g163(.a(new_n257), .b(\b[24] ), .c(new_n258), .d(\b[25] ), .out0(new_n259));
  nano22aa1n03x7               g164(.a(new_n223), .b(new_n236), .c(new_n259), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n256), .c(new_n254), .d(new_n173), .o1(new_n261));
  oao003aa1n02x5               g166(.a(\a[26] ), .b(\b[25] ), .c(new_n249), .carry(new_n262));
  aobi12aa1n06x5               g167(.a(new_n262), .b(new_n243), .c(new_n259), .out0(new_n263));
  xorc02aa1n02x5               g168(.a(\a[27] ), .b(\b[26] ), .out0(new_n264));
  xnbna2aa1n03x5               g169(.a(new_n264), .b(new_n263), .c(new_n261), .out0(\s[27] ));
  norp02aa1n02x5               g170(.a(\b[26] ), .b(\a[27] ), .o1(new_n266));
  inv040aa1n03x5               g171(.a(new_n266), .o1(new_n267));
  inv020aa1n02x5               g172(.a(new_n260), .o1(new_n268));
  aoi012aa1n06x5               g173(.a(new_n268), .b(new_n174), .c(new_n180), .o1(new_n269));
  aoai13aa1n06x5               g174(.a(new_n236), .b(new_n226), .c(new_n210), .d(new_n222), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n259), .o1(new_n271));
  aoai13aa1n06x5               g176(.a(new_n262), .b(new_n271), .c(new_n270), .d(new_n242), .o1(new_n272));
  oaih12aa1n02x5               g177(.a(new_n264), .b(new_n272), .c(new_n269), .o1(new_n273));
  xnrc02aa1n02x5               g178(.a(\b[27] ), .b(\a[28] ), .out0(new_n274));
  tech160nm_fiaoi012aa1n02p5x5 g179(.a(new_n274), .b(new_n273), .c(new_n267), .o1(new_n275));
  aobi12aa1n02x7               g180(.a(new_n264), .b(new_n263), .c(new_n261), .out0(new_n276));
  nano22aa1n03x5               g181(.a(new_n276), .b(new_n267), .c(new_n274), .out0(new_n277));
  norp02aa1n03x5               g182(.a(new_n275), .b(new_n277), .o1(\s[28] ));
  norb02aa1n02x5               g183(.a(new_n264), .b(new_n274), .out0(new_n279));
  oaih12aa1n02x5               g184(.a(new_n279), .b(new_n272), .c(new_n269), .o1(new_n280));
  oao003aa1n02x5               g185(.a(\a[28] ), .b(\b[27] ), .c(new_n267), .carry(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[28] ), .b(\a[29] ), .out0(new_n282));
  tech160nm_fiaoi012aa1n02p5x5 g187(.a(new_n282), .b(new_n280), .c(new_n281), .o1(new_n283));
  aobi12aa1n06x5               g188(.a(new_n279), .b(new_n263), .c(new_n261), .out0(new_n284));
  nano22aa1n03x5               g189(.a(new_n284), .b(new_n281), .c(new_n282), .out0(new_n285));
  norp02aa1n03x5               g190(.a(new_n283), .b(new_n285), .o1(\s[29] ));
  xorb03aa1n02x5               g191(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g192(.a(new_n264), .b(new_n282), .c(new_n274), .out0(new_n288));
  oaih12aa1n02x5               g193(.a(new_n288), .b(new_n272), .c(new_n269), .o1(new_n289));
  oao003aa1n02x5               g194(.a(\a[29] ), .b(\b[28] ), .c(new_n281), .carry(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[29] ), .b(\a[30] ), .out0(new_n291));
  tech160nm_fiaoi012aa1n02p5x5 g196(.a(new_n291), .b(new_n289), .c(new_n290), .o1(new_n292));
  aobi12aa1n02x7               g197(.a(new_n288), .b(new_n263), .c(new_n261), .out0(new_n293));
  nano22aa1n02x4               g198(.a(new_n293), .b(new_n290), .c(new_n291), .out0(new_n294));
  norp02aa1n03x5               g199(.a(new_n292), .b(new_n294), .o1(\s[30] ));
  norb02aa1n02x5               g200(.a(new_n288), .b(new_n291), .out0(new_n296));
  aobi12aa1n02x7               g201(.a(new_n296), .b(new_n263), .c(new_n261), .out0(new_n297));
  oao003aa1n02x5               g202(.a(\a[30] ), .b(\b[29] ), .c(new_n290), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[30] ), .b(\a[31] ), .out0(new_n299));
  nano22aa1n03x5               g204(.a(new_n297), .b(new_n298), .c(new_n299), .out0(new_n300));
  oaih12aa1n02x5               g205(.a(new_n296), .b(new_n272), .c(new_n269), .o1(new_n301));
  tech160nm_fiaoi012aa1n02p5x5 g206(.a(new_n299), .b(new_n301), .c(new_n298), .o1(new_n302));
  norp02aa1n03x5               g207(.a(new_n302), .b(new_n300), .o1(\s[31] ));
  xnrb03aa1n02x5               g208(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  inv000aa1d42x5               g209(.a(new_n103), .o1(new_n305));
  nona22aa1n02x4               g210(.a(new_n106), .b(new_n102), .c(new_n105), .out0(new_n306));
  aoi012aa1n02x5               g211(.a(new_n105), .b(new_n305), .c(new_n104), .o1(new_n307));
  aoi022aa1n02x5               g212(.a(new_n109), .b(new_n305), .c(new_n306), .d(new_n307), .o1(\s[4] ));
  xorb03aa1n02x5               g213(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  inv000aa1d42x5               g214(.a(\a[5] ), .o1(new_n310));
  inv000aa1d42x5               g215(.a(\b[4] ), .o1(new_n311));
  oaoi03aa1n02x5               g216(.a(new_n310), .b(new_n311), .c(new_n109), .o1(new_n312));
  xnrb03aa1n02x5               g217(.a(new_n312), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g218(.a(\a[6] ), .b(\b[5] ), .c(new_n312), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g220(.a(new_n112), .b(new_n314), .c(new_n113), .o1(new_n316));
  xnrb03aa1n02x5               g221(.a(new_n316), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g222(.a(new_n124), .b(new_n140), .c(new_n137), .out0(\s[9] ));
endmodule

